// Bring in my package's API, which is what I'm testing
#include "solver.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout<<"#### in test case 1"<<endl;
//    <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Declare another test
TEST(TestSuite, testCase2)
{
    std::cout<<"#### in test case 1"<<endl;
//    <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
