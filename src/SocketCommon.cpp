#include "SocketCommon.h"

#ifdef WIN32
#    include <atomic>
#endif

namespace aerovista::sync
{
    namespace socket_common
    {
#ifdef WIN32
        std::atomic<int> gWsaRefCount{0};

        void acquireWsa()
        {
            if (gWsaRefCount.fetch_add(1) == 0)
            {
                WSADATA wsainfo;
                WSAStartup(MAKEWORD(2, 2), &wsainfo);
            }
        }

        void releaseWsa()
        {
            if (gWsaRefCount.fetch_sub(1) == 1)
                WSACleanup();
        }
#else
        void acquireWsa()
        {
        }

        void releaseWsa()
        {
        }
#endif
    } // namespace socket_common
} // namespace aerovista::sync
