/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
*/

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace std;

//TODO move this in proper class
void initRectangle(Eigen::Matrix3d K, Eigen::VectorXd& Sw, double lambda,
			Eigen::VectorXd& z, Eigen::VectorXd& shapeParams,
			Eigen::VectorXd& F)
{

	//Get the points
	Eigen::Vector3d m1(z[0], z[1], 1);
	Eigen::Vector3d m2(z[2], z[3], 1);
	Eigen::Vector3d m3(z[4], z[5], 1);
	Eigen::Vector3d m4(z[6], z[7], 1);

	Eigen::Vector3d Ct(Sw[0], Sw[1], Sw[2]);
	Eigen::Quaterniond Cq(Sw[3], Sw[4], Sw[5], Sw[6]);

	//compute normals
	double c2 = (m1.cross(m3).transpose() * m4)[0]
				/ (m2.cross(m3).transpose() * m4)[0];
	double c3 = (m1.cross(m3).transpose() * m2)[0]
				/ (m4.cross(m3).transpose() * m2)[0];

	Eigen::Vector3d n2 = c2 * m2 - m1;
	Eigen::Vector3d n3 = c3 * m4 - m1;

	//Compute rotation matrix columns
	Eigen::Vector3d R1 = K.inverse() * n2;
	R1 = R1 / R1.norm();

	Eigen::Vector3d R2 = K.inverse() * n3;
	R2 = R2 / R2.norm();

	Eigen::Vector3d R3 = R1.cross(R2);

	//Compute frame quaternion
	Eigen::Matrix3d R;
	R << R1, R2, R3;

	const Eigen::Quaterniond qOC(R);

	Eigen::Quaterniond Fqhat = Cq * qOC;

	//Compute frame transaltion
	Eigen::Matrix3d omega = K.transpose().inverse() * K.inverse();
	double ff = sqrt(
				(n2.transpose() * omega * n2)[0]
							/ (n3.transpose() * omega * n3)[0]);

	const Eigen::Vector3d& CtOhat = lambda * K.inverse() * m1;
	const Eigen::Vector3d& Fthat = Ct + Cq.toRotationMatrix() * CtOhat;

	//compute shape parameters
	const Eigen::Vector3d& X = (K * R1);
	const Eigen::Vector3d& Y = (c2 * lambda * m2 - lambda * m1);

	double w = ((X.transpose() * X).inverse() * X.transpose() * Y)[0];
	double h = w / ff;

	//Write the results
	shapeParams << w, h;
	F << Fthat[0], Fthat[1], Fthat[2], Fqhat.w(), Fqhat.x(), Fqhat.y(), Fqhat.z();

}

//TODO move this in ROAMtest
int main(int argc, char *argv[])
{
	Eigen::Matrix3d K;
	K << 500, 0, 250, 0, 500, 250, 0, 0, 1;

	double w = 0.5;
	double h = 0.25;

	Eigen::Vector3d Ft(-0.2, 0.2, 3);
	Eigen::Quaterniond Fq(cos(M_PI / 12), 0, 0, sin(-M_PI / 12));

	Eigen::VectorXd F(7);
	F << Ft[0], Ft[1], Ft[2], Fq.w(), Fq.x(), Fq.y(), Fq.z();

	Eigen::Vector3d Ct(0, 0, 0);
	Eigen::Quaterniond Cq(cos(M_PI / 4), 0, 0, sin(M_PI / 4));
	Eigen::VectorXd Sw(7);
	Sw << Ct[0], Ct[1], Ct[2], Cq.w(), Cq.x(), Cq.y(), Cq.z();

	Eigen::Vector3d M1(0, 0, 0);
	Eigen::Vector3d M2(w, 0, 0);
	Eigen::Vector3d M3(w, h, 0);
	Eigen::Vector3d M4(0, h, 0);

	Eigen::Vector3d m1 = K * Cq.conjugate().toRotationMatrix()
				* ((Ft - Ct) + Fq.toRotationMatrix() * M1);
	Eigen::Vector3d m2 = K * Cq.conjugate().toRotationMatrix()
				* ((Ft - Ct) + Fq.toRotationMatrix() * M2);
	Eigen::Vector3d m3 = K * Cq.conjugate().toRotationMatrix()
				* ((Ft - Ct) + Fq.toRotationMatrix() * M3);
	Eigen::Vector3d m4 = K * Cq.conjugate().toRotationMatrix()
				* ((Ft - Ct) + Fq.toRotationMatrix() * M4);

	m1 = m1 / m1[2];
	m2 = m2 / m2[2];
	m3 = m3 / m3[2];
	m4 = m4 / m4[2];

	Eigen::VectorXd z(8);
	z << m1[0], m1[1], m2[0], m2[1], m3[0], m3[1], m4[0], m4[1];

	Eigen::VectorXd shapeParams(2);
	Eigen::VectorXd Fhat(7);

	initRectangle(K, Sw, 3.0, z, shapeParams, Fhat);

	cout << "F:" << endl << F.transpose() << endl;
	cout << "F^:" << endl << Fhat.transpose() << endl;
	cout << "w, h:" << endl << w << " ," << h << endl;
	cout << "w^, h^:" << endl << shapeParams.transpose() << endl;
}

