#include <winsock2.h>
#include <Ws2bth.h>

int main()
{
SOCKADDR_BTH sockAddr;
SOCKET btSocket;
WORD wVersionRequested;
WSADATA wsaData;
int err;

/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
wVersionRequested = MAKEWORD(2, 2);

err = WSAStartup(wVersionRequested, &wsaData);
if (err != 0) {
    /* Tell the user that we could not find a usable */
    /* Winsock DLL.                                  */
    printf("WSAStartup failed with error: %d\n", err);
    return 1;
}

   btSocket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
   memset (&sockAddr, 0, sizeof(sockAddr));
   sockAddr.addressFamily = AF_BTH;
   sockAddr.serviceClassId = RFCOMM_PROTOCOL_UUID;
   sockAddr.port = BT_PORT_ANY;
   sockAddr.btAddr = <your bluetooth address>
   err = connect(btSocket, (SOCKADDR*)&sockAddr, sizeof(sockAddr));

   return 0;
}
