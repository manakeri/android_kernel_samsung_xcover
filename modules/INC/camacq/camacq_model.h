/*.......................................................................................................
. COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2009           
. All rights are reserved. Reproduction and redistiribution in whole or 
. in part is prohibited without the written consent of the copyright owner.
. 
.   Developer:
.   Date:
.   Description:  
..........................................................................................................
*/

#if !defined(_CAMACQ_MODEL_H_)
#define _CAMACQ_MODEL_H_

/* Define Model */
#if defined(_MAKE_MODEL_JETTA_)
#define CAMACQ_MODEL_JETTA
#elif defined(_MAKE_MODEL_ALKON_)
#define CAMACQ_MODEL_ALKON
#elif defined(_MAKE_MODEL_GFORCE_)
#define CAMACQ_MODEL_GFORCE
#else
#define CAMACQ_MODEL_GFORCE
#endif

#endif /* _CAMACQ_MODEL_H_ */
