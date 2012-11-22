#ifndef _SERIAL_DATA_SERVICE_H_
#define	_SERIAL_DATA_SERVICE_H_

#define PPP_CHAP        0xc223  /* Cryptographic Handshake Auth. Protocol */
#define PPP_PAP         0xc023  /* Password Authentication Protocol */

typedef struct _PapAuthenticationParamsS{
	char	Username[256];
	char	Password[256];
} PapAuthenticationParamsS;


typedef struct _PppAuthenticationParamsS{
	unsigned short auth_type;
	union{
		PapAuthenticationParamsS	*PapAuthenticationParams;
	};
} PppAuthenticationParamsS;

typedef int (*rxCallbackFunc)(const unsigned char *packet, int len, unsigned char cid);
typedef void (*terminateCallbackFunc)(void);
typedef void (*authenticateCallbackFunc) (unsigned char cid , \
	PppAuthenticationParamsS *auth_params);
typedef void (*connectCallbackFunc)(unsigned char cid);

typedef struct _initParams{
	unsigned int		ServiceType;
	rxCallbackFunc		RxCallbackFunc;
	terminateCallbackFunc	TerminateCallbackFunc;
	authenticateCallbackFunc AuthenticateCallbackFunc;
	connectCallbackFunc	ConnectCallbackFunc;
} initParams;


typedef struct _IpConnectionParams {
	unsigned int	IpAddress;
	unsigned int	PrimaryDns;
	unsigned int	SecondaryDns;
} ipConnectionParams;

typedef struct {
  signed int dwContextId;
  signed int dwProtocol;
  struct {
	signed int inIPAddress;
	signed int inPrimaryDNS;
	signed int inSecondaryDNS;
	signed int inDefaultGateway;
	signed int inSubnetMask;
      } ipv4;
 } directIpConfig;

/* PppInit prototype */
typedef void (*initFunc)(initParams *);
/* PppDeinit prototype */
typedef void (*deInitFunc)(void);
/* PppReset prototype */
typedef void (*resetFunc)(void);
/* PppSetCid prototype */
typedef void (*setCidFunc)(unsigned int);
/* PppMessageReq prototype */
typedef void (*messageReqFunc)(unsigned char *, unsigned int);
/* PppUpdateIpParams prototype */
typedef void (*updateIpParamsFunc)(ipConnectionParams *);

typedef struct _dataCBFuncs{
	initFunc init;
	deInitFunc deInit;
	resetFunc reset;
	setCidFunc setCid;
	messageReqFunc messageReq;
	updateIpParamsFunc updateParameters;
} dataCbFuncs;

#define SVC_TYPE_PDP_PPP_MODEM      2


void gs_register_data_service(dataCbFuncs dataCbFuncs);
void gs_unregister_data_service(void);


#endif
