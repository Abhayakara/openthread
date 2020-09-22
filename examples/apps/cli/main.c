/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/platform/logging.h>
#include <openthread/thread.h>
#include <openthread/server.h>

#include "openthread-system.h"
#include "cli/cli_config.h"

#include "srp-api.h"
#include "dns_sd.h"
#include "dns-msg.h"
#include "srp-thread.h"

#define SERVICE_DATA_LEN_BYTES 1
#define SRP_PROXY_SERVICE_ID   0x5D
/** Minimum SRP lease time and default lease time used for a non-sleepy device, in seconds */
#define SRP_MINIMUM_LEASE_TIME 3600
/** SRP key lease time in seconds. Set to one week. */
#define SRP_KEY_LEASE_TIME (60ul * 60 * 24 * 7)

#define BIG_UINT_16(bytes) \
            (uint16_t)( \
            (uint16_t)((uint16_t)((const uint8_t*) (bytes))[0] << 0x08U) | \
            (uint16_t)((uint16_t)((const uint8_t*) (bytes))[1] << 0x00U))

#if OPENTHREAD_EXAMPLES_SIMULATION
#include <setjmp.h>
#include <unistd.h>

jmp_buf gResetJump;

void __gcov_flush();
#endif

#ifndef OPENTHREAD_ENABLE_COVERAGE
#define OPENTHREAD_ENABLE_COVERAGE 0
#endif

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
void *otPlatCAlloc(size_t aNum, size_t aSize)
{
    return calloc(aNum, aSize);
}

void otPlatFree(void *aPtr)
{
    free(aPtr);
}
#endif

void otTaskletsSignalPending(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

static void conflict_callback(const char* hostname) {
    otPlatLog(OT_LOG_LEVEL_CRIT,OT_LOG_REGION_CORE,"hostname conflict %s", hostname );
}

/**
 * Determines whether an address is mesh local or not.  Used to 
 * determine which addresses to advertise over SRP. 
 */
static bool isLocalAddress(otInstance* instance, const otNetifAddress* address) {
    // If it's an RLOC it's definitely mesh local.  Can short circuit other checks quickly here
    if (address->mRloc) {
        return true;
    }

    // Check to see if we match the mesh prefix
    const otMeshLocalPrefix* prefix = otThreadGetMeshLocalPrefix(instance);

    bool match = true;
    for (int i = 0; i < OT_MESH_LOCAL_PREFIX_SIZE && match; i++) {
        match = prefix->m8[i] == address->mAddress.mFields.m8[i];
    }

    return match;
}

/**
 * Looks for srp proxy in the thread network data.  If found 
 * adds server ip addresses to SRP 
 */
static void findSrpProxy(otInstance* instance){
    otServiceConfig serviceConfig;
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;

    while (otNetDataGetNextService(instance, &iterator, &serviceConfig) == OT_ERROR_NONE){
        // Verify Service and Server data lengths, Enterprise number, and Service ID
        if (serviceConfig.mServiceDataLength == SERVICE_DATA_LEN_BYTES &&
            serviceConfig.mEnterpriseNumber == THREAD_ENTERPRISE_NUMBER &&
            serviceConfig.mServiceData[0] == SRP_PROXY_SERVICE_ID &&
            serviceConfig.mServerConfig.mServerDataLength == (OT_IP6_ADDRESS_SIZE + sizeof(uint16_t))) {
            uint16_t portNum = BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[OT_IP6_ADDRESS_SIZE]);
            otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,
                            "srp_add_server_address %04X:%04X:%04X:%04X:%04X:%04X:%04X:%04X - PORT %04X",
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[0]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[2]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[4]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[6]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[8]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[10]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[12]),
                            BIG_UINT_16(&serviceConfig.mServerConfig.mServerData[14]),
                            portNum);
            int srp_err = srp_add_server_address(
                (uint8_t*) &portNum,
                dns_rrtype_aaaa,
                serviceConfig.mServerConfig.mServerData,
                OT_IP6_ADDRESS_SIZE);
            if (srp_err != kDNSServiceErr_NoError) {
                otPlatLog(OT_LOG_LEVEL_CRIT,OT_LOG_REGION_CORE,"ERROR srp_add_server_address: %d", srp_err);
            }

        }
    }
}

/**
 * Checks our IP addresses and adds those visible outside the 
 * mesh to SRP 
 */
static void updateInterfaceAddresses(otInstance* instance){

    // Walk through our addresses
    for (const otNetifAddress* addr = otIp6GetUnicastAddresses(instance); addr; addr = addr->mNext) {
        if (!isLocalAddress(instance, addr)) {
            // Add any addresses that extend beyond mesh-local scope to the interface
            // Print current address
            otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,
                    "srp_add_interface_address %04X:%04X:%04X:%04X:%04X:%04X:%04X:%04X",
                    (addr->mAddress.mFields.m16[0]),
                    (addr->mAddress.mFields.m16[1]),
                    (addr->mAddress.mFields.m16[2]),
                    (addr->mAddress.mFields.m16[3]),
                    (addr->mAddress.mFields.m16[4]),
                    (addr->mAddress.mFields.m16[5]),
                    (addr->mAddress.mFields.m16[6]),
                    (addr->mAddress.mFields.m16[7]));

            int srp_err = srp_add_interface_address(dns_rrtype_aaaa, (uint8_t*) addr->mAddress.mFields.m8, 16);
            if (srp_err != kDNSServiceErr_NoError) {
                otPlatLog(OT_LOG_LEVEL_CRIT,OT_LOG_REGION_CORE,"ERROR srp_add_interface_address: %d", srp_err);
            } 
        }
    }
}

DNSServiceRef dnsService = NULL;
/**
 * Called when Thread network status changes
 * 
 */
void reportStateChanged(uint32_t flags, void* context) {

    OT_UNUSED_VARIABLE(flags);
    OT_UNUSED_VARIABLE(context);
    otInstance* threadInstance = (otInstance*)(context);
    otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"State changed with flags %x", flags);
    static otDeviceRole prevRole = OT_DEVICE_ROLE_DISABLED;
    
    if ( flags & OT_CHANGED_THREAD_ROLE)
    {
        otDeviceRole role = otThreadGetDeviceRole(threadInstance);
        switch (role)
        {
            case OT_DEVICE_ROLE_DISABLED:
                otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread role Disabled");
                break;
            case OT_DEVICE_ROLE_DETACHED:
                otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread role Detached");
                break;
            case OT_DEVICE_ROLE_CHILD:
                otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread role Child");
                break;
            case OT_DEVICE_ROLE_ROUTER:
                otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread role Router");
                break;
            case OT_DEVICE_ROLE_LEADER:
                otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread role Leader");
                break;
        }

        if ( (prevRole == OT_DEVICE_ROLE_DISABLED || prevRole == OT_DEVICE_ROLE_DETACHED ) &&
             (role != OT_DEVICE_ROLE_DISABLED && role != OT_DEVICE_ROLE_DETACHED ) )
        {
            otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Thread Transitioned from disassociated to associated");

            DNSServiceErrorType errorCode = DNSServiceRegister(
                &dnsService,
                /* flags: */ 0,
                /* interfaceIndex */ 0,
                "DemoClient",
                "_hap._udp",
                /* domain: */ NULL,
                /* host: */ NULL,
                /* port*/0,
                /*txtlen*/0, 
                /*txtRecord*/NULL, 
                /*callback*/ NULL,
                /*context*/ NULL);

            otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"DNSService register errorcode %x", errorCode);

        }

        // Update SRP IP addresses
        srp_start_address_refresh();
        updateInterfaceAddresses(threadInstance);
        findSrpProxy(threadInstance);
        srp_finish_address_refresh();
    }
}


int main(int argc, char *argv[])
{
    otInstance *instance;

#if OPENTHREAD_EXAMPLES_SIMULATION
    if (setjmp(gResetJump))
    {
        alarm(0);
#if OPENTHREAD_ENABLE_COVERAGE
        __gcov_flush();
#endif
        execvp(argv[0], argv);
    }
#endif

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

pseudo_reset:

    otSysInit(argc, argv);

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    instance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    instance = otInstanceInitSingle();
#endif
    assert(instance);

#if OPENTHREAD_CONFIG_CLI_TRANSPORT == OT_CLI_TRANSPORT_UART
    otCliUartInit(instance);
#endif

    otSetStateChangedCallback(instance, reportStateChanged, instance);

    // Initialize SRP
    int err = srp_thread_init(instance);
    otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Host init returned %d", err );

    err = srp_set_hostname("DemoClient", conflict_callback);
    otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"Hostname set returned %d", err );

    err = srp_set_lease_times(SRP_KEY_LEASE_TIME, SRP_MINIMUM_LEASE_TIME);
    otPlatLog(OT_LOG_LEVEL_INFO,OT_LOG_REGION_CORE,"lease time set returned %d", err );

    while (!otSysPseudoResetWasRequested())
    {
        otTaskletsProcess(instance);
        otSysProcessDrivers(instance);    
        // Without a Thread Timer API, this hack fakes timer support by 
        // asking SRP to check to see if any of its scheduled processes are ready to fire.
        srp_process_time();
    }

    otInstanceFinalize(instance);
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    free(otInstanceBuffer);
#endif

    goto pseudo_reset;

    return 0;
}

/*
 * Provide, if required an "otPlatLog()" function
 */
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_APP
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);

    va_list ap;
    va_start(ap, aFormat);
    otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    va_end(ap);
}
#endif
