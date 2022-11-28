/*
 * This code implements the MD5 message-digest algorithm.  The algorithm was
 * written by Ron Rivest.  This code was written by Colin Plumb in 1993, our
 * understanding is that no copyright is claimed and that this code is in the
 * public domain.
 *
 * Equivalent code is available from RSA Data Security, Inc.
 * This code has been tested against that, and is functionally equivalent,
 *
 * To compute the message digest of a chunk of bytes, declare an MD5Context
 * structure, pass it to MD5Init, call MD5Update as needed on buffers full of
 * bytes, and then call MD5Final, which will fill a supplied 16-byte array with
 * the digest.
 */

#include <stdint.h>
#include <string.h>
#include "libmd5.h"

//! \ingroup libMD5
//! \{

static void MD5Transform(uint32_t buf[4], uint32_t const in[16]);

#ifndef __BIG_ENDIAN__
# define byteReverse(buf, len)    /* Nothing */
#else
void byteReverse(uint32_t *buf, unsigned len);
/*
 * Note: this code is harmless on little-endian machines.
 */
void byteReverse(uint32_t *buf, unsigned len)
{
  uint32_t t;
  do {
    char* bytes = (char *) buf;
    t = ((unsigned) bytes[3] << 8 | bytes[2]) << 16 |
        ((unsigned) bytes[1] << 8 | bytes[0]);
    *buf = t;
    buf++;
  } while (--len);
}
#endif

/*
 * Start MD5 accumulation.  Set bit count to 0 and buffer to mysterious
 * initialization constants.
 */
void MD5Init(context_md5_t *ctx)
{
  ctx->buf[0] = 0x67452301;
  ctx->buf[1] = 0xefcdab89;
  ctx->buf[2] = 0x98badcfe;
  ctx->buf[3] = 0x10325476;

  ctx->bits[0] = 0;
  ctx->bits[1] = 0;
}

/*
 * Update context to reflect the concatenation of another buffer full
 * of bytes.
 */
void MD5Update(context_md5_t *ctx, unsigned char *buf, unsigned len)
{
  uint32_t t;

  /* Update bitcount */

  t = ctx->bits[0];
  if ((ctx->bits[0] = t + ((uint32_t) len << 3)) < t)
    ctx->bits[1]++;        /* Carry from low to high */
  ctx->bits[1] += len >> 29;

  t = (t >> 3) & 0x3f;    /* Bytes already in shsInfo->data */

  /* Handle any leading odd-sized chunks */

  if (t) {
    unsigned char *p = ctx->in.b8 + t;

    t = 64 - t;
    if (len < t) {
      memcpy(p, buf, len);
      return;
    }
    memcpy(p, buf, t);
    byteReverse(ctx->in.b32, 16);
    MD5Transform(ctx->buf, ctx->in.b32);
    buf += t;
    len -= t;
  }
  /* Process data in 64-byte chunks */

  while (len >= 64) {
    memcpy(ctx->in.b8, buf, 64);
    byteReverse(ctx->in.b32, 16);
    MD5Transform(ctx->buf, ctx->in.b32);
    buf += 64;
    len -= 64;
