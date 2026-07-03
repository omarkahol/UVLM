#include "parser.h"
#include "message.h"
#include "mesher.h"
#include "bladePanel.h"
#include "vtk_formatter.h"
#include "wake.h"
#include "rotation.h"
#include <Eigen/Dense>

int main(int argc, char *argv[])
{	
	VLM::IO::log(std::cout, VLM::IO::welcome_msg);

	//READ THE INPUT FILE AND CHECK ALL THE INPUTS
	VLM::IO::parser reader("config.vlm");
	reader.parse();

	if (! reader.check(std::cout)) {
		return 1;
	}
	VLM::IO::log(std::cout, VLM::IO::parser_msg);

	//DUMP THE PARSER CONTENT INTO THE CONFIG STRUCTS
	VLM::IO::helicopter h = VLM::IO::helicopter_configuration(reader);
	VLM::IO::mechanics m = VLM::IO::mechanics_configuration(reader);
	VLM::IO::simulation s = VLM::IO::simulation_configuration(reader);

	//CREATE THE MESH AS A COLLECTION OF BLADE POINTS
	Eigen::MatrixXd *_pBladeMesh = new Eigen::MatrixXd[h.n_blades];
	//CREATE THE DOF HANDLER FOR THE PANELS
	VLM::mesh::bladePanel *_pBladePanel= new VLM::mesh::bladePanel[h.n_blades*h.n_panels];

	VLM::IO::log(std::cout, VLM::IO::malloc_msg);

	VLM::mesh::create_blade_mesh(_pBladeMesh,h);
	VLM::mesh::create_blade_panels(_pBladePanel,_pBladeMesh, h);
	VLM::IO::log(std::cout, VLM::IO::meshinit_msg);

	VLM::aerodynamics::Wake wake(s.n_collapse, s.n_destroy * h.r);

	// Setup forces CSV
	std::ofstream forces_csv("output/forces.csv");
	forces_csv << "Step,Time,Fx,Fy,Fz" << std::endl;

	Eigen::Vector3d V_inf = Eigen::Vector3d(m.v_inf_x, m.v_inf_y, m.v_inf_z);
	Eigen::Vector3d Omega_vec = Eigen::Vector3d(0, 0, m.omega);

	for (int step = 0; step <= s.nt; ++step) {
		std::cout << "Step: " << step << std::endl;

		if (step > 0) {
			// Advect existing wake
			wake.advect(s.dt, _pBladePanel, h.n_blades * h.n_panels, V_inf);
			
			// Process vortons
			wake.collapse_panels();
			wake.destroy_vortons();

            // Rotate blades
            Eigen::Matrix3d rot = VLM::tensor::R3(m.omega * s.dt);
            for (int i = 0; i < h.n_blades; i++) {
                _pBladeMesh[i] = rot * _pBladeMesh[i];
            }
            // Update panel normals and control points
            for (int i = 0; i < h.n_blades * h.n_panels; i++) {
                _pBladePanel[i].computeNormal();
                _pBladePanel[i].computeControlPoint();
            }

			// Shed new panels (connects new TE to advected old TE)
			wake.shed_from_blade(_pBladePanel, h.n_blades, h.n_panels);
		}

		// Solve for circulation Gamma
		wake.solve_circulation(_pBladePanel, h.n_blades * h.n_panels, V_inf, Omega_vec);

		// Compute forces
		double rho = 1.225; // standard air density
		Eigen::Vector3d total_force = wake.compute_forces(_pBladePanel, h.n_blades * h.n_panels, V_inf, Omega_vec, rho);
		forces_csv << step << "," << step * s.dt << "," 
		           << total_force.x() << "," << total_force.y() << "," << total_force.z() << std::endl;

		if (step % s.itprint == 0) {
			std::string filename = "output/solution_" + std::to_string(step) + ".vtu";
			VLM::IO::vtu_write(_pBladeMesh, _pBladePanel, h, wake, filename, V_inf, Omega_vec);
		}
	}

	forces_csv.close();

	//FREE ALL THE ALLOCATED MEMORY
	delete [] _pBladeMesh;
	delete [] _pBladePanel;
	VLM::IO::log(std::cout, VLM::IO::free_msg);
	return 0;
}