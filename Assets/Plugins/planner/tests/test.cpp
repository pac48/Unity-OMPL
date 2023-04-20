#include <gtest/gtest.h>
#include "library.h"

TEST(OMPL, search) {
    initROS();
    ASSERT_TRUE(true);
    std::cout << "run good" << '\n';

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}