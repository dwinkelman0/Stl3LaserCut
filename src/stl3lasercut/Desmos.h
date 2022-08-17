// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/RingVector.h>
#include <stl3lasercut/Util.h>

#include <fstream>
#include <map>

namespace stl3lasercut {
class AssemblyPlane;
class InterferencePlane;

class DesmosOutput {
 public:
  DesmosOutput(std::ofstream &outputFile,
               const std::shared_ptr<AssemblyPlane> &assembly);
  ~DesmosOutput();

  void outputInterferencePlane(const InterferencePlane &plane);

 private:
  void expr(const std::string &str);
  void expr(const std::optional<std::string> &label,
            const std::string &expression, const uint32_t color = 0);

  void drawPoint(const uint32_t index, const uint32_t color);
  void drawPoint(const std::string &label, const Vec2 point,
                 const uint32_t color);
  void drawLine(const uint32_t source, const uint32_t dest,
                const std::string &label, const uint32_t color);

 private:
  static const RingVector<std::string> AVAILABLE_COLORS;

  std::ostream &outputFile_;
  std::shared_ptr<AssemblyPlane> assembly_;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgeCount_;
};
}  // namespace stl3lasercut
