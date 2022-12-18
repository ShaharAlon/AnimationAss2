#include "BasicScene.h"
#include <igl/writeOBJ.h>




using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)};
    material->AddTexture(0, "textures/box0.bmp", 2);
    boxmat = std::make_shared<Material>("boxmat", program);
    boxmat->AddTexture(0, "textures/bricks.jpg", 2);
    auto fboxmat{ std::make_shared<Material>("fboxmat", program) };
    fboxmat->AddTexture(0, "textures/grass.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto bunnyMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off") };
    Mod1 = Model::Create("mod1", sphereMesh, material);
    Mod2 = Model::Create("mod2", bunnyMesh, material);
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };
    Obj1 = AutoMorphingModel::Create(*Mod1, morphFunc);
    Obj1->showWireframe = true;
    camera->Translate(20, Axis::Z);
    root->AddChild(Obj1);
    Obj2 = AutoMorphingModel::Create(*Mod2, morphFunc);
    Obj2->Scale(5);
    Obj2->showWireframe = true;
    Obj2->Translate({ 3,0,0 });
    root->AddChild(Obj2);
    auto mesh = Obj1->GetMeshList();
    F = mesh[0]->data[0].faces;
    V = mesh[0]->data[0].vertices;
    Tree1.init(Obj1->GetMeshList()[0]->data[0].vertices, Obj1->GetMeshList()[0]->data[0].faces);
    Tree2.init(Obj2->GetMeshList()[0]->data[0].vertices, Obj2->GetMeshList()[0]->data[0].faces);
    igl::edge_flaps(F, E, EMAP, EF, EI);
    Q = {};
    VQ = {};
    C.resize(E.rows(), V.cols());
    EQ = Eigen::VectorXi::Zero(E.rows());
    Eigen::VectorXd costs(E.rows());
    for (int e = 0; e < E.rows(); e++)
    {
        double cost = e;
        Eigen::RowVectorXd p(1, 3);
        igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
        C.row(e) = p;
        costs(e) = cost;
    }
    for (int e = 0; e < E.rows(); e++)
    {
        Q.emplace(costs(e), e, 0);
    }
    boundingBox(Tree1.m_box,Obj1,fboxmat);
    boundingBox(Tree2.m_box,Obj2,fboxmat);
    
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    if (!checkCollision(&Tree1, &Tree2)&&!isCol) {
        Obj2->Translate(velocity, dir);
    }
    else
    {
        isCol = true;
    }


}

void BasicScene::KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_P:
            velocity = velocity / 2;
            break;
        case GLFW_KEY_O:
            velocity = velocity * 2;
            break;
        case GLFW_KEY_1:
            pickedModel=Obj1;
            break;
        case GLFW_KEY_2:
            pickedModel = Obj2;
            break;
        case GLFW_KEY_SPACE:
            Simplification(0.01*Q.size());
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            dir = Axis::Y;
            velocity = abs(velocity);
            break;
        case GLFW_KEY_DOWN:
            dir = Axis::Y;
            velocity = -abs(velocity);
            break;
        case GLFW_KEY_LEFT:
            dir = Axis::X;
            velocity = -abs(velocity);
            break;
        case GLFW_KEY_RIGHT:
            dir = Axis::X;
            velocity = abs(velocity);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_H:
            if(Obj1->meshIndex<=1)
                Obj1->meshIndex = 0;
            else
                Obj1->meshIndex -= 1;
             break;
        case GLFW_KEY_L:
            if (Obj1->meshIndex >= Obj1->GetMeshList()[0]->data.size()-1)
                Obj1->meshIndex = Obj1->GetMeshList()[0]->data.size() - 1;
            else
                Obj1->meshIndex += 1;
            break;
        }
    }
}

void BasicScene::Simplification(int numOfEdges) 
{
    bool anyCollapsed = false;
    auto mesh = Obj1->GetMeshList();
    for (int i = 0; i < numOfEdges; i++) {
        if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
        {
            break;
        }
        anyCollapsed = true;
    }
    if (anyCollapsed) {
        igl::per_vertex_normals(V, F, N);
        T = Eigen::MatrixXd::Zero(V.rows(), 2);
        mesh[0]->data.push_back({ V,F,N,T });
        Obj1->SetMeshList(mesh);
        Obj1->meshIndex = mesh[0]->data.size() - 1;
    }
}

void BasicScene::boundingBox(Eigen::AlignedBox<double, 3> box, std::shared_ptr<cg3d::AutoMorphingModel> obj, std::shared_ptr<cg3d::Material> mat)
{
    Eigen::MatrixXd vb(8, V.cols());

    vb << box.corner(box.BottomLeftFloor).transpose(),
        box.corner(box.BottomRightFloor).transpose(),
        box.corner(box.TopLeftFloor).transpose(),
        box.corner(box.TopRightFloor).transpose(),
        box.corner(box.BottomLeftCeil).transpose(),
        box.corner(box.BottomRightCeil).transpose(),
        box.corner(box.TopLeftCeil).transpose(),
        box.corner(box.TopRightCeil).transpose();

    Eigen::MatrixXi fb(12, F.cols());
    fb << 0, 4, 2,
        6, 4, 2,
        3, 7, 2,
        6, 7, 2,
        1, 3, 5,
        7, 3, 5,
        4, 0, 5,
        1, 0, 5,
        3, 1, 2,
        0, 1, 2,
        7, 6, 5,
        4, 6, 5;
    igl::writeOBJ("data/box.obj", vb, fb);
    auto boxMesh{ IglLoader::MeshFromFiles("box", "data/box.obj") };
    auto bbox = Model::Create("box-"+obj->name, boxMesh, mat);
    bbox->showFaces = false;
    obj->AddChild(bbox);
}

bool BasicScene::checkBoxInter(Eigen::AlignedBox<double, 3> b1, Eigen::AlignedBox<double, 3> b2)
{
    Eigen::Matrix3d A = Obj1->GetRotation().cast<double>().transpose();
    Eigen::Matrix3d B = Obj2->GetRotation().cast<double>().transpose();
    Eigen::Matrix3d C = A.transpose() * B;
    Eigen::Matrix4f Scale1 = Obj1->GetScaling(Obj1->GetTransform()).matrix();
    Eigen::Matrix4f Scale2 = Obj2->GetScaling(Obj2->GetTransform()).matrix();
    Eigen::Vector3d a = Eigen::Vector3d(b1.sizes()[0] * Scale1.row(0)(0), b1.sizes()[1] * Scale1.row(1)(1), b1.sizes()[2] * Scale1.row(2)(2))/2;
    Eigen::Vector3d b = Eigen::Vector3d(b2.sizes()[0] * Scale2.row(0)(0), b2.sizes()[1] * Scale2.row(1)(1), b2.sizes()[2] * Scale2.row(2)(2))/2;
    Eigen::Vector4d Center1 = Eigen::Vector4d(b1.center()[0], b1.center()[1], b1.center()[2],1);
    Eigen::Vector4d Center2 = Eigen::Vector4d(b2.center()[0], b2.center()[1], b2.center()[2],1);
    Eigen::Vector3d D = (Obj2->GetTransform().cast<double>()*Center2 - Obj1->GetTransform().cast<double>()*Center1).head(3);

    for (int i = 0; i < 3; i++)
    {
        if (a(i)+(b(0)*abs(C.row(0)(0))+b(1)*abs(C.row(i)(1))+b(2)*abs(C.row(i)(2))) < abs(A.row(i)*D))
            return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (b(i)+(a(0)*abs(C.row(0)(i))+a(1)*abs(C.row(1)(i))+a(2)*abs(C.row(2)(i))) < abs(B.row(i)*D))
            return false;
    }
    int mat[3][4] = {{1, 2, 2, 1 }, { 0, 2, 2, 0 }, { 0, 1, 1, 0 }};
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(1)(i) * A.row(2) * D;
        R = abs(R - C.row(2)(i) * A.row(1) * D);
        if (a(1) * abs(C.row(2)(i)) + a(2) * abs(C.row(1)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(2)(i) * A.row(0) * D;
        R = abs(R - C.row(0)(i) * A.row(2) * D);
        if (a(0) * abs(C.row(2)(i)) + a(2) * abs(C.row(0)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    for (int i = 0; i < 3; i++)
    {
        double R = C.row(0)(i) * A.row(1) * D;
        R = abs(R - C.row(1)(i) * A.row(0) * D);
        if (a(0) * abs(C.row(1)(i)) + a(1) * abs(C.row(0)(i)) + b(mat[i][0]) * abs(C.row(0)(mat[i][1])) + b(mat[i][2]) * abs(C.row(0)(mat[i][3])) < R)
            return false;
    }
    return true;
}

bool BasicScene::checkCollision(igl::AABB<Eigen::MatrixXd, 3>* t1, igl::AABB<Eigen::MatrixXd, 3>* t2)
{

    if (!checkBoxInter(t1->m_box, t2->m_box)) {
        return false;
    }
    if (t1->is_leaf() && t2->is_leaf()) {
        if (checkBoxInter(t1->m_box, t2->m_box)) {
            if (!isCol) {
                boundingBox(t1->m_box, Obj1,boxmat);
                boundingBox(t2->m_box, Obj2,boxmat);
            }
            return true;
        }
        else 
            return false;
    }
    if (t1->is_leaf()) 
        return checkCollision(t1, t2->m_right)||checkCollision(t1, t2->m_left);

    if (t2->is_leaf()) 
        return checkCollision(t1->m_right,t2)||checkCollision(t1->m_left, t2);
    
    return checkCollision(t1->m_right, t2->m_left)||checkCollision(t1->m_left, t2->m_right)||checkCollision(t1->m_right, t2->m_right)||checkCollision(t1->m_left, t2->m_left);

}



