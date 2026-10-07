#include "fcstub/sha256.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace {

std::string hash_of(const std::string& text) {
    fcstub::Sha256 sha;
    sha.update(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
    return fcstub::Sha256::to_hex(sha.finish());
}

}  // namespace

// NIST FIPS 180-2 / CAVP reference vectors.
TEST(Sha256, EmptyString) {
    EXPECT_EQ(hash_of(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256, Abc) {
    EXPECT_EQ(hash_of("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256, TwoBlockMessage) {
    EXPECT_EQ(hash_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
              "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST(Sha256, MillionA) {
    fcstub::Sha256 sha;
    const std::string chunk(1000, 'a');
    for (int i = 0; i < 1000; ++i) {
        sha.update(reinterpret_cast<const std::uint8_t*>(chunk.data()), chunk.size());
    }
    EXPECT_EQ(fcstub::Sha256::to_hex(sha.finish()),
              "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, ByteWiseFeedEqualsBulkFeed) {
    const std::string text = "The quick brown fox jumps over the lazy dog, repeatedly, 0123456789";
    fcstub::Sha256 bytewise;
    for (const char c : text) {
        const auto b = static_cast<std::uint8_t>(c);
        bytewise.update(&b, 1);
    }
    EXPECT_EQ(fcstub::Sha256::to_hex(bytewise.finish()), hash_of(text));
}
