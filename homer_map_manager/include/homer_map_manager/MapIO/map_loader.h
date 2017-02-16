/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include <homer_map_manager/MapIO/image_loader.h>
#include <nav_msgs/OccupancyGrid.h>
#include "yaml-cpp/yaml.h"

#include <homer_mapnav_msgs/PointOfInterest.h>
#include <homer_mapnav_msgs/RegionOfInterest.h>

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string fname, bool &success);

    nav_msgs::OccupancyGrid getSLAMMap();

    nav_msgs::OccupancyGrid getMaskingMap();

    std::vector<homer_mapnav_msgs::PointOfInterest> getPois();

    std::vector<homer_mapnav_msgs::RegionOfInterest> getRois();

  
  private:

    /** The map data is cached here
     */
    nav_msgs::GetMap::Response map_resp_;
    nav_msgs::OccupancyGrid m_SLAMMap;
    nav_msgs::OccupancyGrid m_MaskingMap;


    std::vector<homer_mapnav_msgs::PointOfInterest> poiList;
    std::vector<homer_mapnav_msgs::RegionOfInterest> roiList;

};
