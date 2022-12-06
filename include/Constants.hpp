#pragma once
#include "SuppressWarningBegin.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "SuppressWarningEnd.hpp"

#include <utility>

constexpr double radiusOf42mm = 0.00425 * 0.5;
constexpr double radiusOf17mm = 0.00168 * 0.5;
constexpr double massOf42mm = 0.041;
constexpr double massOf17mm = 0.0032;

constexpr double widthOfLightBar = 135.0 * 35 / 535;
constexpr double heightOfLightBar = 135.0 * 238 /
    535;  // FIXME: not accurate data directly from user manual, but computed because of lack of documentation.
          // computation is currently based on page 46 RoboMaster 2022 University Series Robot Manufacture specification manual.
          // V1.0（20211015）.pdf by counting the number of light pixels compared to armor pixels in the image inside the pdf.

constexpr double thinnessOfArmor = 0.03;
constexpr double widthOfSmallArmor = 0.135;
constexpr double heightOfSmallArmor = 0.125;
constexpr double widthOfArmorLightBar = 0.02;
constexpr double heightOfArmorLightBar = 0.06;
constexpr double widthOfLargeArmor = 0.230;
constexpr double heightOfLargeArmor = 0.127;

constexpr double radiusOfTriangleArmor = 0.130;

constexpr double maxRelativeSpeedOfArmor = 0.5;
constexpr double maxHeightDifferenceOfArmor = 0.1;

constexpr std::pair<double, double> armorHeightRangeForInfantry = { 0.06, 0.15 };
constexpr std::pair<double, double> armorHeightRangeForBalancedInfantry = { 0.06, 0.40 };
constexpr std::pair<double, double> armorHeightRangeForHero = { 0.06, 0.2 };

constexpr double angleOfArmorForSentry = glm::radians(-15.0);
constexpr double angleOfArmorForInfantry = glm::radians(15.0);

constexpr double radiusOfInfantry = 0.3;
constexpr double radiusOfOutpost = 0.2765;

constexpr double speedThresholdFor17mm = 12.0;
constexpr double speedThresholdFor42mmA = 8.0;
constexpr double speedThresholdFor42mmB = 6.0;
