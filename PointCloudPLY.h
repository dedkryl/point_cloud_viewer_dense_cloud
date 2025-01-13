#pragma once

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

#include <glk/thin_lines.hpp>

#include "PointCloudLAS.h"

enum Mode
{
  POINTS,
  NORMALS,
  INTENSITIES
};


class PointCloudPLY {
public:
  PointCloudPLY(Mode _mode, std::string _filename):
  mode(_mode),fn(_filename)
  {
    auto viewer = guik::LightViewer::instance();
    cube_matrix.reset(new guik::ModelControl("cube_matrix"));
    viewer->register_ui_callback("ui", [this] { ui_callback(); });
    viewer->spin();
    once = true;
  }


private:
  Mode mode;
  std::string fn;
  bool once = false;


  void show_points(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals, std::vector<double>& intens)
  {
    if ((mode == NORMALS) && (!normals.empty())) {
        auto viewer = guik::LightViewer::instance();
         //normals
        std::vector<Eigen::Vector3f> recalc_normals;
        assert(normals.size() == points.size());
        for (size_t i = 0; i < normals.size(); i++)
        {
          auto rc = normals.at(i) + points.at(i);
          recalc_normals.push_back(rc);
        }
   
        //normals, recalculated to her points
        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(recalc_normals);
        viewer->update_drawable("recalc_normals", cloud_buffer, guik::FlatColor(1,1,1));//white

        //points
        auto cloud_buffer_points = std::make_shared<glk::PointCloudBuffer>(points);
        auto shader_setting = guik::FlatGreen().set_point_scale(2.0f);
        viewer->update_drawable("points", cloud_buffer_points, shader_setting);
        //lines bw points and normqals (normals vectors)
        // Line vertices
        std::vector<Eigen::Vector3f> vertices;
        for (size_t i = 0; i < normals.size(); i++)
        {
          vertices.push_back(points.at(i));
          vertices.push_back(recalc_normals.at(i));
        }

        // If line_strip == true, lines are drawn between adjacent vertices (GL_LINE_STRIP).
        // If line_strip == false, lines are drawn between vertices[i * 2] and vertices[i * 2 + 1] (GL_LINES).
        bool line_strip = false;

        // Create lines (All vertices are processed in order)
        //auto lines = std::make_shared<glk::ThinLines>(vertices, line_strip);

        viewer->update_thin_lines("lines", vertices, line_strip, guik::FlatRed());
      }
      else if ((mode == INTENSITIES) && (!intens.empty())) {
        std::cout << " intens " << intens.size() << " points " << points.size() << std::endl; //1-100
        std::vector<Eigen::Matrix<float, 4, 1>> colors;
        for(size_t i= 0; i< points.size();i++)
        {
          Eigen::Vector4f color;

          //if(intens.at(i)>50)
          //  std::cout << i << " : " << intens.at(i) << std::endl;

          //if(points.at(i).x() == 0 && points.at(i).y() == 0 && points.at(i).z() == 0)
          //    std::cout << i << " : zero point " << std::endl;

          color = glk::colormapf(glk::COLORMAP::TURBO, intens.at(i) / 100.0);
          colors.emplace_back(color.cast<float>());
        }
        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points.data(), points.size());
        cloud_buffer->add_color(colors.data(), colors.size());

        auto viewer = guik::viewer();
        viewer->update_drawable("points1", cloud_buffer, guik::VertexColor());
      }  
     // Show points on the viewer
      else if ((mode == POINTS) && (!points.empty())) {
        ///////////////////////////////////////////
        /*
        //additional file reading
        auto ply = glk::load_ply("/home/eugene/ros2_ws/flatwall/flatwall_01_glim_result/INPUT_PLY/glim_ply.ply");
        if (ply) {
                sparse_points.insert(sparse_points.end(), ply->vertices.begin(), ply->vertices.end());
        }

        std::cout << "extended by addtional file sparse_points size = " << sparse_points.size() << std::endl;
        */
        //////////////////////////////////////////

        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
        auto viewer = guik::LightViewer::instance();
        viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      }
      else
        std::cout << "There are no data for selected mode !" << std::endl;

  }
  
  void ui_callback() {

    if(once) return;

    auto viewer = guik::LightViewer::instance();
    points.clear();
    normals.clear();
    intens.clear();

    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Load points
    if(fn.find(".ply") != std::string::npos)//PLY format
    {
            // Load points from PLY and add them to the point list
            viewer->set_title("Reading from PLY file " + fn + " please wait ...");
            auto ply = glk::load_ply(fn);
            if (ply) {
                points.insert(points.end(), ply->vertices.begin(), ply->vertices.end());
            }
    }
    else//LAS format
    {
            using namespace pdal;
            // Set the input point cloud    
            Options read_options;
            viewer->set_title("Reading from LAS file " + fn + " please wait ...");
            read_options.add("filename", fn);
            
            LasReader reader;
            reader.setOptions(read_options);

            // buf_size is the number of points that will be
            // processed and kept in this table at the same time. 
            // A somewhat bigger value may result in some efficiencies.
            int buf_size = 100;
            FixedPointTable t(buf_size);
            reader.prepare(t);

            // Read each point and print it to the screen
            StreamProcessor writer{&points, &normals, &intens};
            Options write_options;
            write_options.add("filename", "stdout");
            writer.setOptions(write_options);
            writer.setInput(reader);
            writer.prepare(t);
            writer.execute(t);

    }
        
    show_points(points, normals, intens);

    std::string smode;
    if(mode == POINTS)
      smode = " (points)";
    else if(mode == INTENSITIES)
      smode = " (intensities)";
    else if(mode == NORMALS)
      smode = " (normals)";

    viewer->set_title("This is  " + fn + smode);

    once = true;
    
    // Show the cube representing the filtering area
    ImGui::Separator();
    cube_matrix->draw_gizmo_ui();
    cube_matrix->draw_gizmo();
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::FlatColor({1.0f, 0.5f, 0.0f, 0.5f}, cube_matrix->model_matrix()).make_transparent());

    ImGui::End();
  }

private:
  std::unique_ptr<guik::ModelControl> cube_matrix;      // Model matrix of the cube representing the filtering area
  std::vector<Eigen::Vector3f> points;                  // Point cloud
  std::vector<Eigen::Vector3f> normals;                  // Point cloud
  std::vector<double> intens;                  // Point cloud
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;  // CloudBuffer of points
};


