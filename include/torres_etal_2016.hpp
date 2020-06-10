/**
 * @file torres_etal_2016.hpp
 * @brief Header file for torres_etal_2016.cpp
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

#ifndef INCLUDED_torres_etal_2016_hpp_
#define INCLUDED_torres_etal_2016_hpp_

// cgutil
#include <cgutil.hpp>

// cpp standard libraries
#include <array>
#include <string>
#include <unordered_map>

// Boost
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/reversed.hpp>

// std_msgs
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

// Service
#include "cpp_uav/Torres16.h"

/**
 * @struct Direction
 * @brief Storage for line sweep direction
 *
 * Sweep direction is from edge to vertex
 */
struct Direction
{
  LineSegment baseEdge;
  geometry_msgs::Point opposedVertex;
};

/**
 * @brief Checks if given path is clockwise (the first turn made to the left) or not
 * @param path
 * @return bool True if path is clockwise
 * @details the definition of "clockwise" is based on Fig.8 in Torres et al. 2016
 */
inline bool isClockWise(const PointVector& path)
{
  return path.at(0).x < path.at(1).x ? true : false;
}

/**
 * @brief Calculates line sweep direction for given polygon
 * @param polygon Line sweep direction is calculated on this region
 * @return direction Struct containing edge and vertex
 */
Direction identifyOptimalSweepDir(const PointVector& polygon, double& distance)
{
  Direction sweepDirection;

  PointVector convexHull = computeConvexHull(polygon);

  // Edges of polygon
  LineSegmentVector edges;

  // Make a list of edges of polygon
  for (std::size_t i = 0; i < convexHull.size(); ++i)
  {
    LineSegment ar;

    ar.at(0) = convexHull.at(i);

    // if vertex is the last one,
    // that vertex makes an edge whose end is the first vertex
    if (i == convexHull.size() - 1)
    {
      ar.at(1) = convexHull.at(0);
    }
    else
    {
      ar.at(1) = convexHull.at(i + 1);
    }
    edges.push_back(ar);
  }

  double min_y, max_y, min_x, max_x;
  int cont = 0;

  // Calculate line sweep direction
  for (const geometry_msgs::Point& vertex : convexHull)
  {
    if(cont = 0){
      min_y = vertex.y;
      max_y = vertex.y;
      min_x = vertex.x;
      max_x = vertex.x;
    }
    if(vertex.y > max_y){
      max_y = vertex.y;
      max_x = vertex.x;
    }
    else if (vertex.y < min_y){
      min_y = vertex.y;
      min_x = vertex.x;
    }
    cont++;
  }
  geometry_msgs::Point p1,p2,p3;
  p1.x = max_x;
  p1.y = max_y;
  p2.x = max_x;
  p2.y = min_y;
  p3.x = min_x;
  p3.y = min_y;
  distance = calculateDistance(p1,p2);
  sweepDirection.opposedVertex = p1;
  LineSegment edge;
  edge[0] = p2;
  edge[1] = p3;
  sweepDirection.baseEdge = edge;
  //std::cout << "Line segment: " << p2.x << "/" << p2.y << " y " << p3.x << "/" << p3.y << std::endl;
  //std::cout << "Opposed vertex: " << p1.x << "/" << p1.y << std::endl;
  return sweepDirection;
}

/**
 * @brief Divided a line given a fixed distance
 * @param p1 point with lower x that defined the line
 * @param p2 point with higher x that defined the line
 * @param step fixed distance between two waypoints
 * @param path Path of coverage path
 */
void divideHorizontalPath(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, PointVector& path, double step)
{
  double distance = calculateDistance(p1,p2);
  int stepNum = std::ceil(distance / step);
  int sign = 1;
  geometry_msgs::Point point;
  if(p1.x > p2.x){
    sign = -1;
  }
  //std::cout << "Puntos generados entre: " << p1.x << "/" << p1.y << " y " << p2.x << "/" << p2.y << std::endl;
  for (int i=1; i <stepNum; i++)
  {
    point.x = p1.x + i*step*sign;
    point.y = p1.y;
    std::cout << point.x << std::endl;
    path.push_back(point);
  }
}

/**
 * @brief Reshape given path
 * @param path The sweep lines of path should be horizontal about x axis
 * @param padding
 * @return PointVector
 * @details Reshape given path so that generated path becomes the sequence of "C" shapes and add padding
 */
PointVector reshapePath(const PointVector& path, double padding, double step)
{
  PointVector zigzagPath;

  // reshape every traverse
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    // even-numbered traverse
    if (i % 2 == 0)
    {
      try
      {
        // in case that the first point of the traverse is located on LEFT side
        if (path.at(2 * i).x < path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          if((p2.x-p1.x) > 2*padding){
            // add padding
            p1.x += padding;
            p2.x -= padding;
            //std::cout << "If 1, linea: " << i << std::endl;
            // be careful with the order of points
            zigzagPath.push_back(p1);
            //divideHorizontalPath(p1, p2, zigzagPath, step);
            zigzagPath.push_back(p2);
          }
          else{
            p1.x = (p1.x + p2.x)/2;
            zigzagPath.push_back(p1);
          }
        }
        // in case that the first point of the traverse is located on RIGHT side
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          if ((p1.x-p2.x)>2*padding){
            // add padding
            p1.x -= padding;
            p2.x += padding;
            //std::cout << "If 2, linea: " << i << std::endl;
            // be careful with the order of points
            zigzagPath.push_back(p2);
            //divideHorizontalPath(p2, p1, zigzagPath, step);
            zigzagPath.push_back(p1);
          }
          else{
            p1.x = (p1.x + p2.x)/2;
            zigzagPath.push_back(p1);
          }
        }
      }
      // in case that the traverse has only one vertex
      catch (std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          // the first vertex of even-numbered traverse of clockwise path is located on RIGHT side of polygon
          p.x += padding;
          zigzagPath.push_back(p);
        }
        else
        {
          // the first vertex of even-numbered traverse of counterclockwise path is located on LEFT side of polygon
          p.x -= padding;
          zigzagPath.push_back(p);
        }
        ROS_ERROR("%s", ex.what());
      }
    }
    // odd-numbered traverse
    else
    {
      try
      {
        // in case that the first point of the traverse is located on RIGHT side
        if (path.at(2 * i).x > path.at(2 * i + 1).x)
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          if((p1.x-p2.x)>2*padding){
            // add padding
            p1.x -= padding;
            p2.x += padding;
            //std::cout << "If 3, linea: " << i << std::endl;
            // be careful with the order of points
            zigzagPath.push_back(p1);
            //divideHorizontalPath(p1, p2, zigzagPath, step);
            zigzagPath.push_back(p2);
          }
          else{
            p1.x = (p1.x + p2.x)/2;
            zigzagPath.push_back(p1);
          }
        }
        // in case that the first point of the traverse is located on LEFT side
        else
        {
          geometry_msgs::Point p1 = path.at(2 * i);
          geometry_msgs::Point p2 = path.at(2 * i + 1);
          if((p2.x-p1.x)>2*padding){
            // add padding
            p1.x += padding;
            p2.x -= padding;
            //std::cout << "If 4, linea: " << i << std::endl;
            // be careful with the order of points
            zigzagPath.push_back(p2);
            //divideHorizontalPath(p2, p1, zigzagPath, step);
            zigzagPath.push_back(p1);
          }
          else{
            p1.x = (p1.x + p2.x)/2;
            zigzagPath.push_back(p1);
          }
        }
      }
      // in case that the traverse has only one vertex
      catch (std::out_of_range& ex)
      {
        geometry_msgs::Point p = path.at(2 * i);
        if (isClockWise(path))
        {
          // the first vertex of odd-numbered traverse of clockwise path is located on LEFT side of polygon
          p.x -= padding;
          zigzagPath.push_back(p);
        }
        else
        {
          // the first vertex of odd-numbered traverse of clockwise path is located on RIGHT side of polygon
          p.x += padding;
          zigzagPath.push_back(p);
        }
        ROS_ERROR("%s", ex.what());
      }
    }
  }
  return zigzagPath;
}

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param sweepDirection
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double footprintLength,
                           double horizontalOverwrap, double verticalOverwrap,
                           const Direction& sweepDirection, double distance, PointVector& path)
{
  // Unable to make polygon with less than 3 points
  if (polygon.size() < 3)
  {
    return false;
  }

  // TODO: Change to configurable
  const double padding = footprintWidth/2.0;
  PointVector rotatedPolygon = polygon;
  // rotate input polygon so that baseEdge become horizontal
  //double rotationAngle = calculateHorizontalAngle(sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back());
  //PointVector rotatedPolygon = rotatePoints(polygon, -rotationAngle);

  // find x coordinate of most left and most right point
  double minX(0), maxX(0);
  for (const auto& vertex : rotatedPolygon)
  {
    if (vertex.x < minX)
    {
      minX = vertex.x;
    }
    else if (vertex.x > maxX)
    {
      maxX = vertex.x;
    }
  }

  double stepWidth = footprintWidth * (1 - horizontalOverwrap);
  double stepLength = footprintLength * (1 - verticalOverwrap);

  // calculate sweep direction of rotated polygon
  PointVector dir{ sweepDirection.opposedVertex, sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back() };
  // dir = rotatePoints(dir, -rotationAngle);
  Direction rotatedDir;
  rotatedDir.opposedVertex = dir.at(0);
  rotatedDir.baseEdge.front() = dir.at(1);
  rotatedDir.baseEdge.back() = dir.at(2);

  //int stepNum = std::ceil(distance / stepWidth);
  //int step_horizonatl = std::ceil ()
  LineSegmentVector sweepLines;
  LineSegment ar;
  geometry_msgs::Point v1,v2,v3;
  v1.x = minX;
  v1.y = rotatedDir.baseEdge.at(0).y + padding;
  v2.x = minX;
  v2.y = rotatedDir.opposedVertex.y - padding;
  v3.x = maxX;
  v3.y = v2.y;

  double distance_ver = calculateDistance(v1,v2);
  int stepNum = std::ceil(distance_ver/stepWidth);
  // generate list of sweep lines which is horizontal against the base edge
  for (int i = 0; i < stepNum; ++i)
  {
    LineSegment ar_;
    geometry_msgs::Point p1, p2;
    p1.x = minX;
    p1.y = rotatedDir.baseEdge.at(0).y + (i * stepWidth) + padding;
    p2.x = maxX;
    p2.y = rotatedDir.baseEdge.at(1).y + (i * stepWidth) + padding;
    //std::cout << "Linea generada en : " << p1.x << "/" << p1.y << " y "<< p2.x << "/" << p2.y << std::endl;
    ar_.at(0) = p1;
    ar_.at(1) = p2;

    sweepLines.push_back(ar_);
  }
  //std::cout << "Linea generada en : " << v2.x << "/" << v2.y << " y "<< v3.x << "/" << v3.y << std::endl;
  ar.at(0) = v2;
  ar.at(1) = v3;
  sweepLines.push_back(ar);
  // Localize intersections of sweeplines and edges of rotated polygon
  LineSegmentVector rotatedEdges = generateEdgeVector(rotatedPolygon, true);

  PointVector intersections;
  std::cout << "intersections" << std::endl;
  for (const auto& sweepLine : sweepLines)
  {
    int intersectionCount = 0;
    for (const auto& edge : rotatedEdges)
    {
      if (hasIntersection(sweepLine, edge))
      {
        intersections.push_back(localizeIntersection(edge, sweepLine));
        std::cout << intersections.back().x << "/" << intersections.back().y << std::endl;
        ++intersectionCount;
      }
      //std::cout << "Number of intersections: " << intersectionCount << std::endl;
      // sweep line in optimal path does not have more than 2 intersections
      if (intersectionCount >= 3)//3
      {
        return false;
      }
    }
  }

  // sort points by y coordinate in ascending order
  std::stable_sort(intersections.begin(), intersections.end(),
                   [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return p1.y < p2.y; });

  const double length_padding = footprintLength/2;
  PointVector rotatedPath = reshapePath(intersections, length_padding, stepLength);

  //path = rotatePoints(rotatedPath, rotationAngle);
  path = rotatedPath;
  //std::cout << "Path size: " << path.size() << std::endl;
  
  if (hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(path, false)) == true)
  {
    std::cout << "Error, path has intersections" << std::endl;
    //return false;
  }

  return true;
}

/**
 * @brief Compute coverage path for convex polygon
 * @param polygon Coverage path is calculated on this region
 * @param footprintWidth Width of the area taken by one sweep
 * @param horizontalOverwrap Horizontal overwrap of each sweep
 * @param path Path of coverage path
 * @return bool True if path does not intersect with polygon
 */
bool computeConvexCoverage(const PointVector& polygon, double footprintWidth, double footprintLength, double horizontalOverwrap, double verticalOverwrap,
                           PointVector& path)
{
  double distance;
  Direction sweepDirection = identifyOptimalSweepDir(polygon, distance);
  return computeConvexCoverage(polygon, footprintWidth, footprintLength, horizontalOverwrap, verticalOverwrap, sweepDirection, distance, path);
}

/**
 * @brief Calculates length of path
 * @param path
 * @return double Length of path
 */
double calculatePathLength(const PointVector& path)
{
  if (path.size() < 2)
  {
    return 0;
  }

  double pathLength = 0;
  for (int i = 0; i < path.size() - 1; ++i)
  {
    pathLength += calculateDistance(path.at(i), path.at(i + 1));
  }
  return pathLength;
}

/**
 * @brief Return counter clock wise-ed path of given path
 * @param path Clockwise path
 * @return PointVector Counter clock wise version of given path
 */
PointVector computeCCWPath(PointVector path)
{
  for (int i = 0; i < std::round(path.size() / 2); ++i)
  {
    // swap the first point and the last point in each sweep line
    geometry_msgs::Point tmp = path.at(2 * i);

    path.at(2 * i) = path.at(2 * i + 1);
    try
    {
      path.at(2 * i + 1) = tmp;
    }
    catch (std::out_of_range& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  return path;
}

/**
 * @brief Return opposite path of given path
 * @param path
 * @return PointVector Path with points of reversed order of given path
 */
PointVector computeOppositePath(const PointVector& path)
{
  PointVector oppositePath;

  // inversely iterate given points
  for (int i = path.size() - 1; i >= 0; --i)
  {
    oppositePath.push_back(path.at(i));
  }

  return oppositePath;
}

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @param end End point
 * @return PointVector Optimal path that minimizes the length of path
 * @details The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const geometry_msgs::Point& start, const geometry_msgs::Point& end)
{
  // The naming of the following variable follows torres et al. 2016
  std::unordered_map<int, std::unordered_map<std::string, geometry_msgs::Point>> coverageAlternatives;
  std::unordered_map<std::string, geometry_msgs::Point> a1, a2, a3, a4;

  PointVector pathCW = isClockWise(path) ? path : computeCCWPath(path);
  PointVector pathCCW = isClockWise(path) ? computeCCWPath(path) : path;

  // a1: clockwise current path
  a1["SP"] = pathCW.front();
  a1["EP"] = pathCW.back();

  // a2: counterclockwise current path
  a2["SP"] = pathCCW.front();
  a2["EP"] = pathCCW.back();

  // a3: clockwise opposite path
  a3["SP"] = pathCW.back();
  a3["EP"] = pathCW.front();

  // a4: counterclockwise opposite path
  a4["SP"] = pathCCW.back();
  a4["EP"] = pathCCW.front();

  coverageAlternatives[1] = a1;
  coverageAlternatives[2] = a2;
  coverageAlternatives[3] = a3;
  coverageAlternatives[4] = a4;

  bool hasIntersectionCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCW, false));
  bool hasIntersectionCCW = hasIntersection(generateEdgeVector(polygon, true), generateEdgeVector(pathCCW, false));

  double minDistance;
  int optimalPath;

  // check which coverage alternative has the shortest path
  for (const auto& coverage : coverageAlternatives | boost::adaptors::indexed())
  {
    // skip calculating length if the path has intersections
    if ((hasIntersectionCW and coverage.index() % 2 == 0) or (hasIntersectionCCW and coverage.index() % 2 != 0))
    {
      continue;
    }

    // only length of transition need to be considered because the length of coverage is almost same
    double distance = calculateDistance(coverage.value().second.at("SP"), start) +
                      calculateDistance(end, coverage.value().second.at("EP"));

    if (distance < minDistance or coverage.index() == 0)
    {
      minDistance = distance;
      optimalPath = coverage.value().first;
    }
  }

  switch (optimalPath)
  {
    case 1:
    {
      //std::cout << "Se devuelve clockwise" << std::endl;
      return pathCW;
    }
    case 2:
    {
      //std::cout << "Se devuelve counterclockwise" << std::endl;
      return pathCCW;
    }
    case 3:
    {
      //std::cout << "Se devuelve opposite clockwise" << std::endl;
      return computeOppositePath(pathCW);
    }
    default:
    {
      //std::cout << "Se devuelve opposite counterclockwise" << std::endl;
      return computeOppositePath(pathCCW);
    }
  }
}

/**
 * @brief Identify optimal path from 4 coverage alternatives
 * @param polygon
 * @param path
 * @param start Start point
 * @return PointVector Optimal path that minimizes the length of path
 * @details The naming of the following variable follows torres et al. 2016
 */
PointVector identifyOptimalAlternative(const PointVector& polygon, const PointVector& path,
                                       const geometry_msgs::Point& start)
{
  return identifyOptimalAlternative(polygon, path, start, start);
}

/**
 * @brief Check if given two polygons are adjacent
 * @param polygon1
 * @param polygon2
 * @return True if given two polygons are adjacent
 */
bool isAdjacent(const PointVector& polygon1, const PointVector& polygon2)
{
  for (const auto& vertex1 : polygon1)
  {
    for (const auto& vertex2 : polygon2)
    {
      // consider that two polygons are adjacent if they have at least one point in common
      if (vertex1 == vertex2)
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Compute coverage path for multiple convex polygons
 * @param subPolygons
 * @param footprintWidth
 * @param horizontalOverwrap
 * @param adjacencyCriteria Ignore paths which have less adjacent polygons than this number
 * @return PointVector Computed Path
 * @details See section 6.2 of torres et al. 2016 for the detail
 */
PointVector computeMultiplePolygonCoverage(std::vector<PointVector> subPolygons, double footprintWidth,
                                           double footprintLength,double horizontalOverwrap, 
                                           double verticalOverwrap, int adjacencyCriteria = 1)
{
  PointVector path;

  std::vector<int> permutation(subPolygons.size());
  std::iota(permutation.begin(), permutation.end(), 0);

  double minPathLength = -1;
  bool hasIntersection = false;

  int numPermutation = 0;
  int numIntersectSets = 0;

  do
  {
    // ignore permutations which do not start from the same polygon as the first one of given subpolygons
    if (permutation.front() != 0)
    {
      continue;
    }

    // count adjacent polygons
    int adjacencyCount = 0;
    for (auto itr = permutation.begin(); itr != permutation.end() - 1; ++itr)
    {
      if (isAdjacent(subPolygons.at(*itr), subPolygons.at(*(itr + 1))) == true)
      {
        ++adjacencyCount;
      }
    }

    // ignore if enough number of polygons do not exist
    if (adjacencyCount < adjacencyCriteria)
    {
      continue;
    }

    double pathLength = 0;
    std::vector<PointVector> candidatePath;

    // computes coverage path for each polygons and connect them
    for (auto itr = permutation.begin(); itr != permutation.end(); ++itr)
    {
      PointVector partPath, optimalAlternative;
      // first polygon of given subpolygon
      if (itr == permutation.begin())
      {
        try
        {
          // start point and end point of first subpolygon are ...
          //    start point: the last point of coverage of the last subpolygon
          //    end point  : the first point of coverage of the second subpolygon
          geometry_msgs::Point start = subPolygons.at(*(permutation.end() - 1)).back();
          geometry_msgs::Point end;
          PointVector polygon = subPolygons.at(*itr);
          if (permutation.size() == 1)
          {
            // end point is the same as start point if subPolygons.at(*(itr+1)) is out of range
            end = start;
          }
          else
          {
            end = subPolygons.at(*(itr + 1)).front();
          }

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth,footprintLength, horizontalOverwrap, verticalOverwrap, partPath) == false)
          {
            hasIntersection = true;
            std::cout << "Path has intersections" << std::endl;
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      // the last polygon of the given subpolygons
      else if (itr == permutation.end() - 1)
      {
        try
        {
          // start point and end point of the last subpolygon are ...
          //    start point: the last point of coverage of the previous subpolygon
          //    end point  : the first point of coverage of the first subpolygon
          geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
          geometry_msgs::Point end = subPolygons.at(*permutation.begin()).front();
          PointVector polygon = subPolygons.at(*itr);

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth, footprintLength, horizontalOverwrap, verticalOverwrap, partPath) == false)
          {
            hasIntersection = true;
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      // middle polygons
      else
      {
        try
        {
          // start point and end point of middle subpolygons are ...
          //    start point: the last point of coverage of the previous subpolygon
          //    end point  : the first point of coverage of the next subpolygon
          geometry_msgs::Point start = subPolygons.at(*(itr - 1)).back();
          geometry_msgs::Point end = subPolygons.at(*(itr + 1)).front();
          PointVector polygon = subPolygons.at(*itr);

          // break if computed path has intersections
          if (computeConvexCoverage(polygon, footprintWidth, footprintLength, horizontalOverwrap, verticalOverwrap, partPath) == false)
          {
            hasIntersection = true;
            break;
          }

          // set optimal alternative as candidate
          optimalAlternative = identifyOptimalAlternative(polygon, partPath, start, end);
          candidatePath.push_back(optimalAlternative);

          // calculate the length of candidate path
          pathLength += calculatePathLength(optimalAlternative);
        }
        catch (std::out_of_range& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
    }

    numPermutation++;

    if(hasIntersection)
    {
      numIntersectSets++;
      hasIntersection = false;
      break;
    }

    // update coverage path and minimum path length
    if (minPathLength < 0 or pathLength < minPathLength)
    {
      minPathLength = pathLength;

      if (not path.empty())
      {
        path.clear();
      }

      // insert coverages of subpolygons
      for (const auto& part : candidatePath)
      {
        path.insert(path.begin(), part.begin(), part.end());
      }
    }

  } while (next_permutation(permutation.begin(), permutation.end()));

  if(minPathLength<0)
  {
    ROS_ERROR("Unable to generate path.");
  }

  return path;
}

#endif
