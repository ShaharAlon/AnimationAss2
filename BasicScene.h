#pragma once

#include "Scene.h"
#include "AutoMorphingModel.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include <igl/collapse_edge.h>
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "per_vertex_normals.h"
#include <AABB.h>


class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void Simplification(int numOfEdges);
    void boundingBox(Eigen::AlignedBox<double, 3> box, std::shared_ptr<cg3d::AutoMorphingModel> obj, std::shared_ptr<cg3d::Material> mat);
    bool checkBoxInter(Eigen::AlignedBox<double, 3> b1, Eigen::AlignedBox<double, 3> b2);
    bool checkCollision(igl::AABB<Eigen::MatrixXd, 3>* t1, igl::AABB<Eigen::MatrixXd, 3>* t2);
private:
    Axis dir=Axis::X;
    float velocity = -0.01;
    std::shared_ptr<cg3d::Material> boxmat;
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> Mod1;
    std::shared_ptr<cg3d::Model> Mod2;
    std::shared_ptr<cg3d::AutoMorphingModel> Obj1;
    std::shared_ptr<cg3d::AutoMorphingModel> Obj2;
    igl::AABB<Eigen::MatrixXd, 3> Tree1;
    igl::AABB<Eigen::MatrixXd, 3> Tree2;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V,C,N,T;
    igl::min_heap<std::tuple<double, int, int>> Q;
    std::vector<std::tuple<int, double>> VQ;
    bool isCol = false;
};
