// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Desmos.h"

#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/InterferencePlane.h>

#include <iomanip>
#include <numbers>
#include <sstream>

namespace stl3lasercut {
DesmosOutput::DesmosOutput(std::ofstream &outputFile,
                           const std::shared_ptr<AssemblyPlane> &assembly)
    : outputFile_(outputFile), assembly_(assembly) {
  outputFile_
      << "<html>\t\n<head>\t\t\n<script "
         "src=\"https://www.desmos.com/api/v1.7/"
         "calculator.js?apiKey=dcb31709b452b1cf9dc26972add0fda6\"></"
         "script>\n\t</head>\n\t<body>\n\t\t<div id=\"calculator\" "
         "style=\"width: 100%; height: 100%;\"></div>\n\t\t<script>\n\t\t\tvar "
         "elt = document.getElementById('calculator'"
         ");\n\t\t\tvar c = "
         "Desmos.GraphingCalculator(elt, {expressionsCollapsed: true});\n";
}

DesmosOutput::~DesmosOutput() {
  outputFile_ << "\t\t</script>\n\t</body>\n</html>";
}

void DesmosOutput::outputInterferencePlane(const InterferencePlane &plane) {
  for (const InterferencePlane::Graph::ConstVertex &vertex :
       plane.graph_.getVertices()) {
    drawPoint(vertex.getIndex(), 0);
  }
  for (const auto &[line, group] : plane.groupMap_) {
    std::stringstream ss;
    ss << *group;
    for (auto it = group->points.begin(),
              end = std::next(group->points.end(), -1);
         it != end; ++it) {
      for (const auto &edge : group->edges) {
        drawLine(*it, *std::next(it), ss.str(), edge.color);
      }
    }
  }
}

void DesmosOutput::expr(const std::string &str) {
  outputFile_ << "\t\t\t" << str << "\n";
}

void DesmosOutput::expr(const std::optional<std::string> &label,
                        const std::string &expression, const uint32_t color) {
  static uint32_t idCounter = 0;
  outputFile_ << "\t\t\tc.setExpression({id: 'element" << idCounter++
              << "', latex: '" << expression << "', ";
  if (label) {
    outputFile_ << "label: '" << *label << "', showLabel: true, ";
  }
  outputFile_ << "color: '" << AVAILABLE_COLORS[color] << "'});\n";
}

void DesmosOutput::drawPoint(const uint32_t index, const uint32_t color) {
  std::stringstream ss;
  ss << "v" << index;
  drawPoint(ss.str(), assembly_->getPoint(index), color);
}

void DesmosOutput::drawPoint(const std::string &label, const Vec2 point,
                             const uint32_t color) {
  std::stringstream ss;
  ss << std::fixed << "(" << std::get<0>(point) << ", " << std::get<1>(point)
     << ")";
  expr(label, ss.str(), color);
}

void DesmosOutput::drawLine(const uint32_t source, const uint32_t dest,
                            const std::string &label, const uint32_t color) {
  const float OFFSET = -0.02;
  uint32_t count =
      edgeCount_.emplace(std::pair<uint32_t, uint32_t>(source, dest), 0)
          .first->second++;
  Vec2 a = assembly_->getPoint(source);
  Vec2 b = assembly_->getPoint(dest);
  std::stringstream ss;
  ss << std::fixed;
  DirectedLine line = DirectedLine::fromPoints(a, b)->getParallelLineWithOffset(
      count * OFFSET + OFFSET / 2);
  a = *line.getIntersection(line.getPerpendicularLineThroughPoint(a, true));
  b = *line.getIntersection(line.getPerpendicularLineThroughPoint(b, true));
  ss << line;
  if (std::get<0>(line.getDirectionVector()) == 1) {
    ss << " \\\\{" << std::min(std::get<1>(a), std::get<1>(b)) << " < y < "
       << std::max(std::get<1>(a), std::get<1>(b)) << "\\\\}";
  } else {
    ss << " \\\\{" << std::min(std::get<0>(a), std::get<0>(b)) << " < x < "
       << std::max(std::get<0>(a), std::get<0>(b)) << "\\\\}";
  }
  expr(label, ss.str(), color);
}

const RingVector<std::string> DesmosOutput::AVAILABLE_COLORS({"black", "blue",
                                                              "orange", "green",
                                                              "red", "purple"});
}  // namespace stl3lasercut
