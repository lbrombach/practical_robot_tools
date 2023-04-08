#include <gtest/gtest.h>
#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/SerialMessageTypes.h"
#include <vector>

TEST(ros_arduino, emtpy_msg_test)
{
    std::cout<<"\nempty msg test"<<std::endl;
    // create test message
    SerialMessage<bool> a(MSG_TYPE_EMPTY, "test_empty");
    a.show();

}

TEST(ros_arduino, bool_test)
{
    std::cout<<"\nbool test"<<std::endl;
    //create test message
    SerialMessage<bool> a(MSG_TYPE_BOOL, "test_bool");
    a.data.resize(5);
    a.data.assign({0, 1, 1, 0, 1});
    a.show();

    // Pack the SerialMessage object into a vector to send over serial
    std::vector<uint8_t> packet = a.pack();

    // Unpack the vector into a new SerialMessage object
    SerialMessage<bool> b;
    b = b.unpack(packet);
    b.show();

    // Check that the original and unpacked SerialMessage objects are the same
    ASSERT_TRUE(a==b);
}

TEST(ros_arduino, uint8_test)
{
    std::cout<<"\nuint8 test"<<std::endl;
    // Create a SerialMessage object
    SerialMessage<uint8_t> a(MSG_TYPE_UINT8, "test_uint8");
    a.data.resize(11);
    a.data.assign({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255});
    a.show();

    // Pack the SerialMessage object into a vector to send over serial
    std::vector<uint8_t> packet = a.pack();

    // Unpack the vector into a new SerialMessage object
    SerialMessage<uint8_t> b;
    b = b.unpack(packet);
    b.show();

    // Check that the original and unpacked SerialMessage objects are the same
    ASSERT_TRUE(a==b);
}

TEST(ros_arduino, int8_test)
{
    std::cout<<"\nint8 test"<<std::endl;
    // Create a SerialMessage object
    SerialMessage<int8_t> a(MSG_TYPE_INT8, "test_int8");
    a.data.resize(11);
    a.data.assign({-128, 1, -2, 3, 4, 0, 6, 7, -8, 9, 127});
    a.show();

    // Pack the SerialMessage object into a vector to send over serial
    std::vector<uint8_t> packet = a.pack();

    // Unpack the vector into a new SerialMessage object
    SerialMessage<int8_t> b;
    b = b.unpack(packet);
    b.show();

    // Check that the original and unpacked SerialMessage objects are the same
    ASSERT_TRUE(a==b);
}

TEST(ros_arduino, int16_test)
{
    std::cout<<"\nint16 test"<<std::endl;
    // Create a SerialMessage object
    SerialMessage<int16_t> a(MSG_TYPE_INT16, "test_int16");
    a.data.resize(11);
    a.data.assign({INT16_MIN, 768, 1, 2, 3, 4, 0, 6, 7, 8, 9, 32, INT16_MAX});
    a.show();

    // Pack the SerialMessage object into a vector to send over serial
    std::vector<uint8_t> packet = a.pack();

    // Unpack the vector into a new SerialMessage object
    SerialMessage<int16_t> b;
    b = b.unpack(packet);
    b.show();

    // Check that the original and unpacked SerialMessage objects are the same
    ASSERT_TRUE(a==b);
}

TEST(ros_arduino, int32_test)
{
    std::cout<<"\nint32 test"<<std::endl;
    // Create a SerialMessage object
    SerialMessage<int32_t> a(MSG_TYPE_INT32, "test_int32");
    a.data.resize(11);
    a.data.assign({INT32_MIN, 768, 1, 2, 3, 4, 0, 6, 7, 8, 9, 32, INT32_MAX});
    a.show();

    // Pack the SerialMessage object into a vector to send over serial
    std::vector<uint8_t> packet = a.pack();

    // Unpack the vector into a new SerialMessage object
    SerialMessage<int32_t> b;
    b = b.unpack(packet);
    b.show();

    // Check that the original and unpacked SerialMessage objects are the same
    ASSERT_TRUE(a==b);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}