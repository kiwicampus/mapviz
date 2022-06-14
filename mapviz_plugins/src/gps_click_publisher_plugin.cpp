// *****************************************************************************
//
// Copyright (c) 2014-2020, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/gps_click_publisher_plugin.h>
#include <swri_transform_util/frames.h>
#include <tf2/transform_datatypes.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ Standard Libraries
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::GpsClickPublisherPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  GpsClickPublisherPlugin::GpsClickPublisherPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , canvas_(nullptr)
  {
    ui_.setupUi(config_widget_);

    connect(&click_filter_, SIGNAL(pointClicked(const QPointF&)),
            this, SLOT(pointClicked(const QPointF&)));
    connect(ui_.topic, SIGNAL(textEdited(const QString&)),
            this, SLOT(topicChanged(const QString&)));

    frame_timer_.start(1000);
    connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(updateFrames()));
  }

  GpsClickPublisherPlugin::~GpsClickPublisherPlugin()
  {
    if (canvas_)
    {
      canvas_->removeEventFilter(&click_filter_);
    }
  }

  bool GpsClickPublisherPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    canvas_->installEventFilter(&click_filter_);

    PrintInfo("Looking for wgs84 transform...");

    return true;
  }

  void GpsClickPublisherPlugin::Draw(double x, double y, double scale)
  {
  }

  void GpsClickPublisherPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string tmp;
    if (node["topic"])
    {
      tmp = node["topic"].as<std::string>();
      ui_.topic->setText(QString(tmp.c_str()));
      topicChanged(ui_.topic->text());
    }

  }

  void GpsClickPublisherPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
  }

  QWidget* GpsClickPublisherPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }


  void GpsClickPublisherPlugin::pointClicked(const QPointF& point)
  {
    QPointF transformed = canvas_->MapGlCoordToFixedFrame(point);

    std::string output_frame = "wgs84";

    if(target_frame_.empty())
    {
      PrintError("Fixed frame not set");
      return;
    }

    swri_transform_util::Transform tf;
    tf2::Vector3 tfPoint(transformed.x(), transformed.y(), 0.0);
    if (tf_manager_->GetTransform(output_frame, target_frame_, tf))
    {
      tfPoint = tf * tfPoint;
    } else {
      std::stringstream error;
      error << "Unable to find transform from " << target_frame_ << " to " << output_frame << ".";
      PrintError(error.str());
      return;
    }
    transformed.setX(tfPoint.x());
    transformed.setY(tfPoint.y());
 

    std::unique_ptr<geographic_msgs::msg::GeoPoint> geopoint =
      std::make_unique<geographic_msgs::msg::GeoPoint>();
    geopoint->longitude = transformed.x();
    geopoint->latitude = transformed.y();
    geopoint->altitude = 0.0;

    std::stringstream ss;
    ss << "Point in " << output_frame.c_str() << ": " << transformed.x() << "," << transformed.y();

    // Only publish if this plugin is visible
    if(this->Visible())
    {
      point_publisher_->publish(*geopoint);
    }
    else
    {
      ss << " (but not publishing since plugin is hidden)";
    }
    
    PrintInfo(ss.str());
  }

  void GpsClickPublisherPlugin::SetNode(rclcpp::Node& node)
  {
    mapviz::MapvizPlugin::SetNode(node);

    // We override this method so that we can initialize our publisher after
    // our node has been set, ensuring that it's in mapviz's namespace.
    topicChanged(ui_.topic->text());
  }

  void GpsClickPublisherPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void GpsClickPublisherPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void GpsClickPublisherPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }


  void GpsClickPublisherPlugin::topicChanged(const QString& topic)
  {
    std::stringstream ss;
    ss << "Publishing points to topic: " << topic.toStdString().c_str();
    PrintInfo(ss.str());

    if (!topic.isEmpty())
    {
      point_publisher_ = node_->create_publisher<geographic_msgs::msg::GeoPoint>(
          topic.toStdString(), rclcpp::QoS(1000));
    }
  }

  void GpsClickPublisherPlugin::updateFrames()
  {
    bool supports_wgs84 = tf_manager_->SupportsTransform(
        swri_transform_util::_local_xy_frame,
        swri_transform_util::_wgs84_frame);

    if (!supports_wgs84)
    {
      PrintError("wgs84 frame unavailable. Is GPS being published?");
      return;
    }
    if(target_frame_.empty())
    {
      PrintError("Fixed frame not set");
      return;
    }
      
  }
}   // namespace mapviz_plugins
