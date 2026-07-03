#include "bladePanel.h"

VLM::mesh::bladePanel::bladePanel(Eigen::MatrixXd *mesh, int globalDoF[4]) {
    this->mesh=mesh;
    for(int i=0; i<4; i++) {
        this->globalDoF[i] = globalDoF[i];
    }
}
VLM::mesh::bladePanel::~bladePanel() {
    mesh = nullptr;
}
Eigen::Vector3d VLM::mesh::bladePanel::getPoint(const int &p) {
    if (p<1 || p>4) {
        throw "illegal point";
    }
    return mesh->col(this->globalDoF[p-1]);
}
void VLM::mesh::bladePanel::computeNormal() {
    this->n=(getPoint(3)-getPoint(2)).cross(getPoint(1)-getPoint(2));
    this->n.normalize();
}
void VLM::mesh::bladePanel::computeControlPoint() {
    this->cp = 0.5*( 0.5*(getPoint(1)+getPoint(2)) + 0.5*(getPoint(3)+getPoint(4)));
}
Eigen::Vector3d VLM::mesh::bladePanel::getControlPoint() {
    return this->cp;
}
Eigen::Vector3d VLM::mesh::bladePanel::getNormal() {
    return this->n;
}