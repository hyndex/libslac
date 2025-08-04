#ifndef SLAC_ENDIAN_HPP
#define SLAC_ENDIAN_HPP

#include <cstdint>

#ifdef htole16
#undef htole16
#undef le16toh
#endif
#ifdef htole32
#undef htole32
#undef le32toh
#endif
#ifdef htole64
#undef htole64
#undef le64toh
#endif
#ifdef htons
#undef htons
#undef ntohs
#endif

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#endif

#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN 4321
#endif

#if defined(_WIN32)
#ifndef __BYTE_ORDER
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif
#elif defined(__BYTE_ORDER__) && !defined(__BYTE_ORDER)
#define __BYTE_ORDER __BYTE_ORDER__
#elif defined(__BYTE_ORDER)
/* already defined */
#elif defined(__LITTLE_ENDIAN__) || defined(__ARMEL__) || defined(__MIPSEL__)
#define __BYTE_ORDER __LITTLE_ENDIAN
#else
#define __BYTE_ORDER __BIG_ENDIAN
#endif

namespace slac {

static inline constexpr uint16_t bswap16(uint16_t v) {
    return __builtin_bswap16(v);
}
static inline constexpr uint32_t bswap32(uint32_t v) {
    return __builtin_bswap32(v);
}
static inline constexpr uint64_t bswap64(uint64_t v) {
    return __builtin_bswap64(v);
}


#if !defined(ESP_PLATFORM) && !defined(htole16)
inline constexpr uint16_t htole16(uint16_t v) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return v;
#else
    return bswap16(v);
#endif
}
inline constexpr uint16_t le16toh(uint16_t v) { return htole16(v); }
#endif


#if !defined(ESP_PLATFORM) && !defined(htole32)
inline constexpr uint32_t htole32(uint32_t v) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return v;
#else
    return bswap32(v);
#endif
}
inline constexpr uint32_t le32toh(uint32_t v) { return htole32(v); }
#endif

#if !defined(ESP_PLATFORM) && !defined(htole64)
inline constexpr uint64_t htole64(uint64_t v) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return v;
#else
    return bswap64(v);
#endif
}
inline constexpr uint64_t le64toh(uint64_t v) { return htole64(v); }
#endif

#if !defined(ESP_PLATFORM) && !defined(htons)
inline constexpr uint16_t htons(uint16_t v) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return bswap16(v);
#else
    return v;
#endif
}
inline constexpr uint16_t ntohs(uint16_t v) { return htons(v); }
#endif

} // namespace slac

#endif // SLAC_ENDIAN_HPP
