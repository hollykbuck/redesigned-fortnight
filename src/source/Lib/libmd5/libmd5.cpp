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
  }

    /* Handle any remaining bytes of data. */

  memcpy(ctx->in.b8, buf, len);
}

/*
 * Final wrapup - pad to 64-byte boundary with the bit pattern
 * 1 0* (64-bit count of bits processed, MSB-first)
 */
void MD5Final(unsigned char digest[16], context_md5_t *ctx)
{
  unsigned count;
  unsigned char *p;

  /* Compute number of bytes mod 64 */
  count = (ctx->bits[0] >> 3) & 0x3F;

  /* Set the first char of padding to 0x80.  This is safe since there is
     always at least one byte free */
  p = ctx->in.b8 + count;
  *p++ = 0x80;

  /* Bytes of padding needed to make 64 bytes */
  count = 64 - 1 - count;

  /* Pad out to 56 mod 64 */
  if (count < 8) {
    /* Two lots of padding:  Pad the first block to 64 bytes */
    memset(p, 0, count);
    byteReverse(ctx->in.b32, 16);
    MD5Transform(ctx->buf, ctx->in.b32);

    /* Now fill the next block with 56 bytes */
    memset(ctx->in.b8, 0, 56);
  } else {
    /* Pad block to 56 bytes */
    memset(p, 0, count - 8);
  }
  byteReverse(ctx->in.b32, 14);

  /* Append length in bits and transform */
  ctx->in.b32[14] = ctx->bits[0];
  ctx->in.b32[15] = ctx->bits[1];

  MD5Transform(ctx->buf, ctx->in.b32);
  byteReverse((uint32_t *) ctx->buf, 4);
  memcpy(digest, ctx->buf, 16);

  memset(ctx, 0, sizeof(* ctx));    /* In case it's sensitive */
  /* The original version of this code omitted the asterisk. In
     effect, only the first part of ctx was wiped with zeros, not
     the whole thing. Bug found by Derek Jones. Original line: */
  // memset(ctx, 0, sizeof(ctx));    /* In case it's sensitive */
}

/* The four core functions - F1 is optimized somewhat */

/* #define F1(x, y, z) (x & y | ~x & z) */
#define F1(x, y, z) (z ^ (x & (y ^ z)))
#define F2(x, y, z) F1(z, x, y)
#define F3(x, y, z) (x ^ y ^ z)
#define F4(x, y, z) (y ^ (x | ~z))

/* This is the central step in the MD5 algorithm. */
#define MD5STEP(f, w, x, y, z, data, s) \
    ( w += f(x, y, z) + data,  w = w<<s | w>>(32-s),  w += x )

/*
 * The core of the MD5 algorithm, this alters an existing MD5 hash to
 * reflect the addition of 16 longwords of new data.  MD5Update blocks
 * the data and converts bytes into longwords for this routine.
 */
static void MD5Transform(uint32_t buf[4], uint32_t const in[16])
{
  register uint32_t a, b, c, d;

  a = buf[0];
  b = buf[1];
  c = buf[2];
  d = buf[3];

  MD5STEP(F1, a, b, c, d, in[0] + 0xd76aa478, 7);
  MD5STEP(F1, d, a, b, c, in[1] + 0xe8c7b756, 12);
  MD5STEP(F1, c, d, a, b, in[2] + 0x242070db, 17);
  MD5STEP(F1, b, c, d, a, in[3] + 0xc1bdceee, 22);
  MD5STEP(F1, a, b, c, d, in[4] + 0xf57c0faf, 7);
  MD5STEP(F1, d, a, b, c, in[5] + 0x4787c62a, 12);
  MD5STEP(F1, c, d, a, b, in[6] + 0xa8304613, 17);
  MD5STEP(F1, b, c, d, a, in[7] + 0xfd469501, 22);
  MD5STEP(F1, a, b, c, d, in[8] + 0x698098d8, 7);
  MD5STEP(F1, d, a, b, c, in[9] + 0x8b44f7af, 12);
  MD5STEP(F1, c, d, a, b, in[10] + 0xffff5bb1, 17);
  MD5STEP(F1, b, c, d, a, in[11] + 0x895cd7be, 22);
  MD5STEP(F1, a, b, c, d, in[12] + 0x6b901122, 7);
  MD5STEP(F1, d, a, b, c, in[13] + 0xfd987193, 12);
  MD5STEP(F1, c, d, a, b, in[14] + 0xa679438e, 17);
  MD5STEP(F1, b, c, d, a, in[15] + 0x49b40821, 22);

