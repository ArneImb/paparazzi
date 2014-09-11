

#include "protocol.h"



/** Receiving pprz messages */

// PPRZ parsing state machine
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4

struct mora_transport mora_protocol;

static inline void parse_mora(struct mora_transport * t, uint8_t c ) {
  switch (t->status) {
  case UNINIT:
    if (c == STX)
      t->status++;
    break;
  case GOT_STX:
    if (t->msg_received) {
      t->error++;
      goto error;
    }
    t->payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    t->ck_a = t->ck_b = c;
    t->status++;
    t->payload_idx = 0;
    break;
  case GOT_LENGTH:
    t->payload[t->payload_idx] = c;
    t->ck_a += c; t->ck_b += t->ck_a;
    t->payload_idx++;
    if (t->payload_idx == t->payload_len)
      t->status++;
    break;
  case GOT_PAYLOAD:
    if (c != t->ck_a)
      goto error;
    t->status++;
    break;
  case GOT_CRC1:
    if (c != t->ck_b)
      goto error;
    t->msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  t->error++;
 restart:
  t->status = UNINIT;
  return;
}


/*
static inline void pprz_parse_payload(struct pprz_transport * t) {
  uint8_t i;
  for(i = 0; i < t->trans.payload_len; i++)
    dl_buffer[i] = t->trans.payload[i];
  dl_msg_available = TRUE;
}


#define PprzBuffer(_dev) TransportLink(_dev,ChAvailable())
#define ReadPprzBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_pprz(&(_trans),TransportLink(_dev,Getch())); }
#define PprzCheckAndParse(_dev,_trans) {  \
  if (PprzBuffer(_dev)) {                 \
    ReadPprzBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {      \
      pprz_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;  \
    }                                     \
  }                                       \
}
*/

