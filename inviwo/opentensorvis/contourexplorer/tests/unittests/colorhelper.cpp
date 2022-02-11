#include <warn/push>
#include <warn/ignore/all>
#include <gtest/gtest.h>
#include <warn/pop>

#include <inviwo/contourexplorer/util/segmentationcolorhelper.h>

namespace inviwo {
//TEST(ColorHelperTests, ThreeItems) {
//    constexpr size_t n = 3;
//
//    const auto positionsAndAlphas = SegmentationColorHelper::getPositionsAndLevelsForNSegments(n);
//
//    constexpr auto third = 1.0 / 3.0;
//    constexpr auto offset = std::numeric_limits<double>::epsilon();
//
//    const std::vector<std::pair<double, double>> reference{
//        {third - 2 * offset, 1.0},   {third - offset, 0.0},           {third + offset, 0.0},
//        {third + 2 * offset, 1.0},   {2.0 * third - 2 * offset, 1.0}, {2.0 * third - offset, 0.0},
//        {2.0 * third + offset, 0.0}, {2.0 * third + 2 * offset, 1.0}};
//
//    EXPECT_EQ(positionsAndAlphas, reference);
//}
//
//TEST(ColorHelperTests, FourItems) {
//    constexpr size_t n = 4;
//
//    const auto positionsAndAlphas = SegmentationColorHelper::getPositionsAndLevelsForNSegments(n);
//
//    constexpr auto fourth = 1.0 / 4.0;
//    constexpr auto offset = std::numeric_limits<double>::epsilon();
//
//    const std::vector<std::pair<double, double>> reference{
//        {fourth - 2.0 * offset, 1.0},    {fourth - offset, 0.0},
//
//        {fourth + offset, 0.0},         {fourth + 2.0 * offset, 1.0},
//        {2.0 * fourth - 2.0 * offset, 1.0}, {2.0 * fourth - offset, 0.0},
//
//        {2.0 * fourth + offset, 0.0},           {2.0 * fourth + 2.0 * offset, 1.0},
//        {3 * fourth - 2.0 * offset, 1.0}, {3.0 * fourth - offset, 0.0},
//
//        {3.0 * fourth + offset, 0.0},        {3.0 * fourth + 2.0 * offset, 1.0}};
//
//    EXPECT_EQ(positionsAndAlphas, reference);
//}
}  // namespace inviwo