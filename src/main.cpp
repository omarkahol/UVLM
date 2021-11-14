#include "parser.h"
#include "message.h"
#include "mesher.h"
#include "bladePanel.h"
#include "vtk_formatter.h"


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
	arma::mat *_pBladeMesh = new arma::mat[h.n_blades];
	//CREATE THE DOF HANDLER FOR THE PANELS
	VLM::mesh::bladePanel *_pBladePanel= new VLM::mesh::bladePanel[h.n_blades*h.n_panels];

	VLM::IO::log(std::cout, VLM::IO::malloc_msg);

	VLM::mesh::create_blade_mesh(_pBladeMesh,h);
	VLM::mesh::create_blade_panels(_pBladePanel,_pBladeMesh, h);
	VLM::IO::log(std::cout, VLM::IO::meshinit_msg);

	VLM::IO::vtu_write(_pBladeMesh,_pBladePanel,h,"mesh.vtu");

	//FREE ALL THE ALLOCATED MEMORY
	delete [] _pBladeMesh;
	delete [] _pBladePanel;
	VLM::IO::log(std::cout, VLM::IO::free_msg);
	return 0;
}