
#ifdef NS3_MODULE_COMPILATION
# error "Do not include ns3 module aggregator headers from other modules; these are meant only for end user scripts."
#endif

#ifndef NS3_MODULE_VLC
    

// Module headers:
#include "vlc-channel-helper.h"
#include "vlc-channel-model.h"
#include "vlc-device-helper.h"
#include "vlc-error-model.h"
#include "vlc-mobility-model.h"
#include "vlc-net-device.h"
#include "vlc-propagation-loss-model.h"
#include "vlc-rx-net-device.h"
#include "vlc-snr.h"
#include "vlc-tx-net-device.h"
#endif
