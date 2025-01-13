#pragma once

#include <io/LasReader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <iostream>

namespace pdal
{

// A class to read a point cloud from a file point by point, without
// loading it fully in memory. The points are printed to the screen,
// but this can be replaced with any other processing. See the LasWriter
// class for a more detailed example.
 
class PDAL_EXPORT StreamProcessor: public Writer, public Streamable
{

public:
    std::string getName() const;
    StreamProcessor(std::vector<Eigen::Vector3f>* points, std::vector<Eigen::Vector3f>* normals, std::vector<double>* intens);
    ~StreamProcessor();

private:

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void writeView(const PointViewPtr view);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);
    StreamProcessor& operator=(const StreamProcessor&) = delete;
    StreamProcessor(const StreamProcessor&) = delete;
    StreamProcessor(const StreamProcessor&&) = delete;
    std::vector<Eigen::Vector3f>* _points;
    std::vector<Eigen::Vector3f>* _normals;
    std::vector<double>* _intens;

};

std::string StreamProcessor::getName() const { return "sample streamer"; }

StreamProcessor::StreamProcessor(std::vector<Eigen::Vector3f>* points, std::vector<Eigen::Vector3f>* normals, std::vector<double>* intens):
    _points(points),_normals(normals),_intens(intens)
 {}

StreamProcessor::~StreamProcessor() {}

void StreamProcessor::addArgs(ProgramArgs& args)
{
}

void StreamProcessor::initialize()
{
}

// This will be called for each point in the cloud.
bool StreamProcessor::processOne(PointRef& point)
{
    /*
    std::cout << "Point: " 
    << "GpsTime: " << point.getFieldAs<double>(Dimension::Id::GpsTime) <<  '\t' 
    //<< "has X: " << point.hasDim(Dimension::Id::X) <<  '\t' 
    << "X: " << point.getFieldAs<double>(Dimension::Id::X) <<  '\t' 
    << "Y: " << point.getFieldAs<double>(Dimension::Id::Y) << '\t' 
    << "Z: " << point.getFieldAs<double>(Dimension::Id::Z) << '\t'
    //<< "has Intensity: " << point.hasDim(Dimension::Id::Intensity) <<  '\t' 
    << "Intensity: " << point.getFieldAs<double>(Dimension::Id::Intensity) <<  '\t'
    //<< "has NormalX: " << point.hasDim(Dimension::Id::NormalX) <<  '\t'  
    << "NormalX: " << point.getFieldAs<double>(Dimension::Id::NormalX) <<  '\t' 
    << "NormalY: " << point.getFieldAs<double>(Dimension::Id::NormalY) << '\t' 
    << "NormalZ: " << point.getFieldAs<double>(Dimension::Id::NormalZ)
    << std::endl;
    */

    Eigen::Vector3f p;
    p[0] = point.getFieldAs<double>(Dimension::Id::X);
    p[1] = point.getFieldAs<double>(Dimension::Id::Y);
    p[2] = point.getFieldAs<double>(Dimension::Id::Z);
   _points->push_back(p);

    Eigen::Vector3f n;
    n[0] = point.getFieldAs<double>(Dimension::Id::NormalX);
    n[1] = point.getFieldAs<double>(Dimension::Id::NormalY);
    n[2] = point.getFieldAs<double>(Dimension::Id::NormalZ);
   _normals->push_back(n);

    double i;
    i = point.getFieldAs<double>(Dimension::Id::Intensity);
    _intens->push_back(i);

    return true;  
}

void StreamProcessor::done(PointTableRef table) {
}

void StreamProcessor::writeView(const PointViewPtr view)
{
    throw pdal_error("The writeView() function must not be called in streaming mode.");
}

} // namespace pdal
