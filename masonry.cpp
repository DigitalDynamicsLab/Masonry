//
// MASONRY
//
// Program that simulates masonry using the Chrono::Engine multibody library
// from www.chronoengine.info
//

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/collision/ChCCollisionSystemBullet.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/utils/ChCompositeInertia.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional> 
#include <cctype>
#include <unordered_map>


// Use the namespace of Chrono

using namespace chrono;
using namespace irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


//using namespace std;


// Some global variables

int    GLOBAL_save_each = 10;
int    GLOBAL_snapshot_each = 0;
double GLOBAL_max_simulation_time = 30;
bool   GLOBAL_load_forces = true; 
bool   GLOBAL_swap_zy = false;
double GLOBAL_density = 1800;
float  GLOBAL_friction = 0.4f;
float  GLOBAL_damping =  0.2f;
float  GLOBAL_compliance = 2e-8f;
float  GLOBAL_rolling_friction = 0;
float  GLOBAL_spinning_friction = 0;
float  GLOBAL_rolling_compliance = 0;
float  GLOBAL_spinning_compliance = 0;
double GLOBAL_penetrationrecovery = 0.001;
bool   GLOBAL_warmstart = false;

std::shared_ptr<ChFunction_Recorder> GLOBAL_motion_X; // motion on x direction
std::shared_ptr<ChFunction_Recorder> GLOBAL_motion_Y; // motion on y (vertical) direction
std::shared_ptr<ChFunction_Recorder> GLOBAL_motion_Z; // motion on z direction
double GLOBAL_motion_timestep = 0.01; // timestep for sampled earthquake motion 
double GLOBAL_motion_amplifier = 1.0; // scale x,y,z motion by this factor
double GLOBAL_timestep = 0.01; // timestep for timestepper integrator
bool GLOBAL_use_motions = false;
int GLOBAL_iterations = 500;

double GLOBAL_totmass = 0;

// Load brick pattern from disk
// Create a bunch of ChronoENGINE rigid bodies 

void load_brick_file(ChSystem& mphysicalSystem, const char* filename, 
                    std::shared_ptr<ChMaterialSurface> mmaterial, 
                    std::unordered_map<int, std::shared_ptr<ChBody>>& my_body_map,
					bool do_collide) {

    GetLog() << "Parsing " << filename << " brick file... \n";

	GLOBAL_totmass = 0;

    std::fstream fin(filename);
	if (!fin.good())
		throw ChException("ERROR opening .dat file with bricks: " + std::string(filename) + "\n");

    int added_bricks = 0;

    std::string line;
    
    // Parse the file line-by-line
	while(std::getline(fin, line)) 
	{
		//trims white space from the beginning of the string
		line.erase(line.begin(), find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace)))); 
        
        // skip empty lines
		if(line[0] == 0) 
            continue; 

        // skip comments
        if(line[0] == '#') {
			continue; 
		}

		// a normal line should contain brick data:
		if (true)
		{
			std::vector<double> tokenvals;
            std::vector<bool>   tokenasterisk;
			int ntokens = 0;

			std::string token;
			std::istringstream ss(line);

            // parse line in format:
            // ID, fixed, visible, Fx Fy, Fz, Ref.x, Ref.y, Ref.z, x,y,z, x,y,z, x,y,z, ..,..,..
			while(std::getline(ss, token,',') && ntokens < 300) 
			{
                tokenvals.push_back(0);
                tokenasterisk.push_back(false);

				std::istringstream stoken(token);
                //GetLog() << "  token n." << ntokens << " is: "<< stoken.str().c_str() << "\n";
				if (token == "*") {
                    tokenasterisk[ntokens] = true; 
                    tokenvals[ntokens] = 0;
                }
                else {
                    tokenasterisk[ntokens] = false;
                    stoken >> tokenvals[ntokens]; 
                }
				++ntokens;	
			}
			++added_bricks;

            int  my_ID    = (int)tokenvals[0];
            bool my_fixed = (bool)tokenvals[1]; 
            bool my_visible = (bool)tokenvals[2]; 
            int token_stride = 3;

            ChVector<> my_force;
            if (GLOBAL_load_forces) {
                my_force.x() = tokenvals[token_stride+0];
                my_force.y() = tokenvals[token_stride+1];
                my_force.z() = tokenvals[token_stride+2];
                token_stride += 3;
                if (GLOBAL_swap_zy) std::swap(my_force.y(), my_force.z());
            }

            ChVector<> my_reference;
            my_reference.x() = tokenvals[token_stride+0];
            my_reference.y() = tokenvals[token_stride+1];
            my_reference.z() = tokenvals[token_stride+2];
            if (GLOBAL_swap_zy) std::swap(my_reference.y(), my_reference.z());
            token_stride += 3;
            
            std::vector< std::vector< ChVector<> > > my_vertexes;
            my_vertexes.push_back(std::vector< ChVector<> >());
            while (true) {
                if (token_stride+2 >= ntokens) {
                    throw ChException("ERROR in .dat file, format is: ID, fixed, visible, Fx, Fy, Fz, Refx,Refy,Refz, and three x y z coords, each per brick corner, see line:\n"+ line+"\n");
                    break;
                }
                ChVector<> my_point;
                my_point.x() = tokenvals[token_stride+0];
                my_point.y() = tokenvals[token_stride+1];
                my_point.z() = tokenvals[token_stride+2];

                if (GLOBAL_swap_zy) std::swap(my_point.y(), my_point.z());
                my_point = my_point - my_reference; // chrono want these points in reference system, but in file are in absolute system
                my_vertexes.back().push_back(my_point);
                token_stride += 3;

				if (token_stride == ntokens)
					break;

                if (tokenasterisk[token_stride] == true) {
                    token_stride +=1; // skip asterisk separator, if any
                    my_vertexes.push_back(std::vector< ChVector<> >()); // begin other list of convex hulls
                }
                
            }

            // Create a polygonal body:
            //std::shared_ptr<ChBodyEasyConvexHullAuxRef> my_body (new ChBodyEasyConvexHullAuxRef(my_vertexes[0],1800,true, my_visible));
            std::shared_ptr<ChBodyAuxRef> my_body (new ChBodyAuxRef);

            my_body->GetCollisionModel()->ClearModel();

            utils::CompositeInertia composite_inertia;

            for (int ih = 0 ; ih < my_vertexes.size() ; ++ih) {

                auto vshape = std::make_shared<ChTriangleMeshShape>();
                collision::ChConvexHullLibraryWrapper lh;
                lh.ComputeHull(my_vertexes[ih], *vshape->GetMesh());
                if (my_visible) {
                    my_body->AddAsset(vshape);
                }

                double i_mass;
                ChVector<> i_baricenter;
                ChMatrix33<> i_inertia;
                vshape->GetMesh()->ComputeMassProperties(true, i_mass, i_baricenter, i_inertia);

                composite_inertia.AddComponent(ChFrame<>(i_baricenter), i_mass, i_inertia);

                // if collide required
                if (true) {
                    // avoid passing to collision the inner points discarded by convex hull
                    // processor, so use mesh vertexes instead of all argument points
                    std::vector<ChVector<> > points_reduced;
                    points_reduced.resize(vshape->GetMesh()->getCoordsVertices().size());
                    for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
                        points_reduced[i] = vshape->GetMesh()->getCoordsVertices()[i];

                    my_body->GetCollisionModel()->AddConvexHull(points_reduced);  
                }
            }

            ChMatrix33<> principal_inertia_csys;
            double principal_I[3];
            composite_inertia.GetInertia().FastEigen(principal_inertia_csys, principal_I);

            my_body->SetDensity((float)GLOBAL_density);
            my_body->SetMass(composite_inertia.GetMass() * GLOBAL_density);
            my_body->SetInertiaXX(ChVector<>(principal_I[0] * GLOBAL_density, principal_I[1] * GLOBAL_density, principal_I[2] * GLOBAL_density));

			if (!my_fixed) {
				GLOBAL_totmass += my_body->GetMass();
			}
			//GetLog() << " block mass=" << my_body->GetMass() << "   volume=" << composite_inertia.GetMass() << "\n";

            // Set the COG coordinates to barycenter, without displacing the REF reference
            my_body->SetFrame_COG_to_REF(ChFrame<>(composite_inertia.GetCOM(), principal_inertia_csys));


            my_body->GetCollisionModel()->BuildModel();
			if (do_collide)
				my_body->SetCollide(true);

            my_body->SetIdentifier(my_ID);
            my_body->Set_Scr_force(my_force);
            my_body->SetMaterialSurface(mmaterial);
            my_body->SetFrame_REF_to_abs(ChFrame<>(my_reference));

            // Set the color of body by randomizing a gray shade
            std::shared_ptr<ChColorAsset> mcolor (new ChColorAsset);
            double mgray = 0.6+0.4*ChRandom();
            mcolor->SetColor(ChColor(mgray, mgray, mgray));
            my_body->AddAsset(mcolor);

            mphysicalSystem.Add(my_body);

            // Fix to ground 
            if(my_fixed) {
                if (GLOBAL_use_motions) {
                    // earthquake motion required: so "fixed" means "impose motion respect to a fixed body"
                    std::shared_ptr<ChBody> absolute_body (new ChBody);
                    absolute_body->SetBodyFixed(true);
                    mphysicalSystem.Add(absolute_body);

                    std::shared_ptr<ChLinkLockLock> link_earthquake (new ChLinkLockLock);
                    link_earthquake->Initialize(my_body, absolute_body, ChCoordsys<>(my_body->GetPos()) );
                    link_earthquake->SetMotion_X(GLOBAL_motion_X);
                    link_earthquake->SetMotion_Y(GLOBAL_motion_Y);
                    link_earthquake->SetMotion_Z(GLOBAL_motion_Z);

                    mphysicalSystem.Add(link_earthquake);
                }
                else {
                    // simplier approach: no earthquake motion, just fix body
                    my_body->SetBodyFixed(true);
                }
            }
            
            // add body to map
            my_body_map[my_ID] = my_body;

		}


	} // end while

    GetLog() << " ...ok, parsed " << filename << " brick file successfully, created " << added_bricks << " bricks.\n";
	GetLog() << " TOTAL MASS OF NOT FIXED BRICKS: " << GLOBAL_totmass << "\n\n";
}


// Load springs from disk

void load_spring_file(ChSystem& mphysicalSystem, std::string& filename, std::unordered_map<int, std::shared_ptr<ChBody>>& my_body_map) {

    GetLog() << "Parsing " << filename << " spring file... \n";

    std::fstream fin(filename);
	if (!fin.good())
		throw ChException("ERROR opening .dat file with springs: " + filename + "\n");

    int added_springs = 0;

    std::string line;
    
    // Parse the file line-by-line
	while(std::getline(fin, line)) 
	{
		//trims white space from the beginning of the string
		line.erase(line.begin(), find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace)))); 
        
        // skip empty lines
		if(line[0] == 0) 
            continue; 

        // skip comments
        if(line[0] == '#') {
			continue; 
		}

		// a normal line should contain spring data:
		if (true)
		{
			double tokenvals[300];
			int ntokens = 0;

			std::string token;
			std::istringstream ss(line);

            // parse line in format:
            // ID, IDbodyA, x,y,z, IDbodyB, x,y,z,  k, L0
			while(std::getline(ss, token,',') && ntokens < 300) 
			{
				std::istringstream stoken(token);
				stoken >> tokenvals[ntokens]; 
				++ntokens;	
			}
			++added_springs;

            if (ntokens != 11)
                throw ChException("ERROR in .dat file of springs, format is: ID, IDbodyA, x,y,z, IDbodyB, x,y,z,  k, L0 :\n"+ line+"\n");

            int  my_ID      = (int)tokenvals[0];

            int my_IDbodyA = (int)tokenvals[1]; 
            if (my_body_map.find(my_IDbodyA) == my_body_map.end())
                throw ChException("ERROR in .dat file of springs, body with identifier bodyA=" + std::to_string(my_IDbodyA) +  " not found :\n"+ line+"\n");
            ChVector<> my_referenceA;
            my_referenceA.x() = tokenvals[2];
            my_referenceA.y() = tokenvals[3];
            my_referenceA.z() = tokenvals[4];
            if (GLOBAL_swap_zy) std::swap(my_referenceA.y(), my_referenceA.z());

            int my_IDbodyB = (int)tokenvals[5];  
            if (my_body_map.find(my_IDbodyB) == my_body_map.end())
                throw ChException("ERROR in .dat file of springs, body with identifier bodyB=" + std::to_string(my_IDbodyB) +  " not found :\n"+ line+"\n");
            ChVector<> my_referenceB;
            my_referenceB.x() = tokenvals[6];
            my_referenceB.y() = tokenvals[7];
            my_referenceB.z() = tokenvals[8];
            if (GLOBAL_swap_zy) std::swap(my_referenceB.y(), my_referenceB.z());
            
            double my_k   = tokenvals[9];
            double my_L0 =  tokenvals[10];
            
            // Create a spring:
            std::shared_ptr<ChLinkSpring> my_spring (new ChLinkSpring());
            my_spring->SetIdentifier(my_ID);
            std::shared_ptr<ChBody> mbodyA = my_body_map[my_IDbodyA];
            std::shared_ptr<ChBody> mbodyB = my_body_map[my_IDbodyB];
            GetLog() << "mbodyA ID: " << mbodyA->GetIdentifier() << " for ID " << my_IDbodyA << "  pos: " << mbodyA->GetPos() << "\n";
            GetLog() << "mbodyB ID: " << mbodyB->GetIdentifier() << " for ID " << my_IDbodyB << "  pos: " << mbodyB->GetPos() << "\n";
            my_spring->Initialize(mbodyA, mbodyB, false, my_referenceA, my_referenceB, false, my_L0);
            my_spring->Set_SpringK(my_k);
            mphysicalSystem.Add(my_spring);

            std::shared_ptr<ChPointPointSegment> my_line (new ChPointPointSegment());
            my_line->SetColor(ChColor(1,0,0));
            my_spring->AddAsset(my_line);
		}

	} // end while

    GetLog() << " ...ok, parsed " << filename << " spring file successfully, created " << added_springs  << " springs.\n";
}


// Load precomputed contact positions from disk
class PrecomputedContact {
public:
	PrecomputedContact(const std::shared_ptr<ChBody> mbodyA, const std::shared_ptr<ChBody> mbodyB, const ChVector<> mpos_t0, const ChVector<> mnormal_t0)
		: bodyA(mbodyA), bodyB(mbodyB), pos_t0(mpos_t0), normal_t0(mnormal_t0)
	{
		rel_pos_A = bodyA->TransformPointParentToLocal(pos_t0);
		rel_pos_B = bodyB->TransformPointParentToLocal(pos_t0);
		rel_normal_A = bodyA->TransformDirectionParentToLocal(normal_t0);
		reaction_cache[0] = 0;
		reaction_cache[1] = 0;
		reaction_cache[2] = 0;
	}

	// For time different than t0, absolute position of contact points might change from the mpos_t0 initial
	// value, ex. they can detach or compenetrate a bit, so this can be used to retrieve the two points given
	// the current position of the two blocks.
	void GetAbsolutePoints(ChVector<>& abs_pos_A, ChVector<>& abs_pos_B, ChVector<>& abs_norm_A) {
		abs_pos_A = bodyA->TransformPointLocalToParent(rel_pos_A);
		abs_pos_B = bodyB->TransformPointLocalToParent(rel_pos_B);
		abs_norm_A = bodyA->TransformDirectionLocalToParent(rel_normal_A);
	}

	collision::ChCollisionInfo GetCollisionInfo() {
		collision::ChCollisionInfo minfo;
		minfo.modelA = bodyA->GetCollisionModel().get();
		minfo.modelB = bodyB->GetCollisionModel().get();
		minfo.vpA = bodyA->TransformPointLocalToParent(rel_pos_A);
		minfo.vpB = bodyB->TransformPointLocalToParent(rel_pos_B);
		minfo.vN  = bodyA->TransformDirectionLocalToParent(rel_normal_A);
		minfo.distance = Vdot(minfo.vpB - minfo.vpA, minfo.vN);
		minfo.reaction_cache = this->reaction_cache;
		return minfo;
	}

//private:
	float reaction_cache[3];
	ChVector<> pos_t0;
	ChVector<> normal_t0;
	std::shared_ptr<ChBody> bodyA;
	std::shared_ptr<ChBody> bodyB;
	ChVector<> rel_pos_A;
	ChVector<> rel_pos_B;
	ChVector<> rel_normal_A;
};

void load_contacts_file(ChSystem& mphysicalSystem, std::string& filename, std::unordered_map<int, std::shared_ptr<ChBody>>& my_body_map, std::vector<PrecomputedContact>& mprecomputed_contacts) {

	GetLog() << "Parsing " << filename << " file of precomputed contacts... \n";

	std::fstream fin(filename);
	if (!fin.good())
		throw ChException("ERROR opening .dat file with precomputed contacts: " + filename + "\n");

	int added_contacts = 0;

	std::string line;

	// Parse the file line-by-line
	while (std::getline(fin, line))
	{
		//trims white space from the beginning of the string
		line.erase(line.begin(), find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));

		// skip empty lines
		if (line[0] == 0)
			continue;

		// skip comments
		if (line[0] == '#') {
			continue;
		}

		// a normal line should contain contact data:
		if (true)
		{
			double tokenvals[300];
			int ntokens = 0;

			std::string token;
			std::istringstream ss(line);

			// parse line in format:
			// IDbodyA, x,y,z, IDbodyB, x,y,z,  k, L0
			while (std::getline(ss, token, ',') && ntokens < 300)
			{
				std::istringstream stoken(token);
				stoken >> tokenvals[ntokens];
				++ntokens;
			}
			++added_contacts;

			if (ntokens != 8)
				throw ChException("ERROR in .dat file of contacts, format is: IDbodyA, IDbodyB, x,y,z, Nx,Ny,Nz but here is:\n" + line + "\n");

			int my_IDbodyA = (int)tokenvals[0];
			if (my_body_map.find(my_IDbodyA) == my_body_map.end())
				throw ChException("ERROR in .dat file of contacts, body with identifier bodyA=" + std::to_string(my_IDbodyA) + " not found :\n" + line + "\n");

			int my_IDbodyB = (int)tokenvals[1];
			if (my_body_map.find(my_IDbodyB) == my_body_map.end())
				throw ChException("ERROR in .dat file of contacts, body with identifier bodyB=" + std::to_string(my_IDbodyB) + " not found :\n" + line + "\n");

			ChVector<> my_abspos;
			my_abspos.x() = tokenvals[2];
			my_abspos.y() = tokenvals[3];
			my_abspos.z() = tokenvals[4];
			if (GLOBAL_swap_zy) std::swap(my_abspos.y(), my_abspos.z());

			ChVector<> my_absnorm;
			my_absnorm.x() = tokenvals[5];
			my_absnorm.y() = tokenvals[6];
			my_absnorm.z() = tokenvals[7];
			if (GLOBAL_swap_zy) std::swap(my_absnorm.y(), my_absnorm.z());

			// store the contact info 
			std::shared_ptr<ChBody> mbodyA = my_body_map[my_IDbodyA];
			std::shared_ptr<ChBody> mbodyB = my_body_map[my_IDbodyB];

			PrecomputedContact mcontact(mbodyA, mbodyB, my_abspos, my_absnorm.GetNormalized() );
			mprecomputed_contacts.push_back(mcontact);
		}

	} // end while

	GetLog() << " ...ok, parsed " << filename << " contacts file successfully, created " << added_contacts << " contacts.\n";
}



// Load seismic displacement function
// from ascii file, each row is a value followed by CR. Time step is assumed constant.

void load_motion(std::shared_ptr<ChFunction_Recorder> mrecorder, std::string filename_pos, double t_offset = 0, double factor =1.0, double timestep = 0.01)
{
    GetLog() << "Parsing " << filename_pos << " motion file... \n";

	ChStreamInAsciiFile mstream(filename_pos.c_str());
	
	mrecorder->Reset();

	double time = 0;
	while(!mstream.End_of_stream())
	{
		double value = 0;
		try
		{
			//mstream >> time;
			mstream >> value;

			//GetLog() << "  t=" << time + t_offset << "  p=" << value * factor << "\n";

			mrecorder->AddPoint(time + t_offset, value * factor);
            time += timestep;
		}
		catch(ChException myerror)
		{
			GetLog() << "  End parsing file " << filename_pos.c_str() << " because: \n  " << myerror.what() << "\n";
			break;
		}
	}
	GetLog() << " ...ok, parsed " << filename_pos << " motion file successfully: " << mrecorder->GetPoints().size() << " samples with dt=" << timestep << "\n";
}





// This is the contact reporter class, just for writing contacts on 
// a file on disk
class _contact_reporter_class : public  ChContactContainer::ReportContactCallback
{
    public:
    ChStreamOutAsciiFile* mfile; // the file to save data into

	virtual bool OnReportContact(
		const ChVector<>& pA,             ///< contact pA
		const ChVector<>& pB,             ///< contact pB
		const ChMatrix33<>& plane_coord,  ///< contact plane coordsystem (A column 'X' is contact normal)
		const double& distance,           ///< contact distance
		const double& eff_radius,         ///< effective radius of curvature at contact
		const ChVector<>& react_forces,   ///< react.forces (if already computed). In coordsystem 'plane_coord'
		const ChVector<>& react_torques,  ///< react.torques, if rolling friction (if already computed).
		ChContactable* contactobjA,  ///< model A (note: some containers may not support it and could be nullptr)
		ChContactable* contactobjB   ///< model B (note: some containers may not support it and could be nullptr)
	) override {

        // For each contact, this function is executed. 
        // In this example, saves on ascii file:
        //   position xyz, direction xyz, normal impulse, tangent impulse U, tangent impulse V, modelA ID, modelB ID information is saved. 
        (*mfile)    << contactobjA->GetPhysicsItem()->GetIdentifier() << ", "
                    << contactobjB->GetPhysicsItem()->GetIdentifier() << ", "    
                    << pA.x() << ", " 
                    << pA.y() << ", " 
                    << pA.z() << ", " 
                    << react_forces.x() << ", "
                    << react_forces.y() << ", "
                    << react_forces.z() << ", "
                    << plane_coord.Get_A_Xaxis().x() << ", "
                    << plane_coord.Get_A_Xaxis().y() << ", "
                    << plane_coord.Get_A_Xaxis().z() << ", "
                    << plane_coord.Get_A_Yaxis().x() << ", "
                    << plane_coord.Get_A_Yaxis().y() << ", "
                    << plane_coord.Get_A_Yaxis().z() << ", "
                    << plane_coord.Get_A_Zaxis().x() << ", "
                    << plane_coord.Get_A_Zaxis().y() << ", "
                    << plane_coord.Get_A_Zaxis().z() << "\n";
        /*
        GetLog() << "ReportContactCallback! \n";
        GetLog() << plane_coord;
        GetLog() << " dot product between X and Y ="<< Vdot(plane_coord.Get_A_Xaxis(), plane_coord.Get_A_Yaxis()) << "\n";
        GetLog() << " dot product between Y and Z ="<< Vdot(plane_coord.Get_A_Yaxis(), plane_coord.Get_A_Zaxis()) << "\n\n";
        */

        return true;  // to continue scanning contacts
    }
};



/// THE PROGRAM STARTS HERE!!!

int main(int argc, char* argv[]) {

    GLOBAL_motion_X = std::make_shared<ChFunction_Recorder>();
    GLOBAL_motion_Y = std::make_shared<ChFunction_Recorder>();
    GLOBAL_motion_Z = std::make_shared<ChFunction_Recorder>();

    // Parse input command

    char* filename = "bricks.dat"; // commento per Vale: variabile stringa, inizializzata a default
    std::string file_motion_x = "";
    std::string file_motion_y = "";
    std::string file_motion_z = "";
    std::string file_springs  = "";
	std::string file_contacts = "";
	bool use_SOR_solver = false;

    if (argc >=2)
        filename = argv[1];

    int iarg = 2;
    while(iarg+1 < argc) {
        bool got_command = false;
        std::string command  = argv[iarg];
        std::string argument = argv[iarg+1];
        if (command == "motion_X") {
            got_command = true;
            file_motion_x = argument;
        }
        if (command == "motion_Y") {
            got_command = true;
            file_motion_y = argument;
        }
        if (command == "motion_Z") {
            got_command = true;
            file_motion_z = argument;
        }
		if (command == "precomputed_contacts") {
			got_command = true;
			file_contacts = argument;
		}
        if (command == "motion_dt")  {
            got_command = true;
            GLOBAL_motion_timestep = atof(argument.c_str());
        }
        if (command == "motion_amplifier")  {
            got_command = true;
            GLOBAL_motion_amplifier = atof(argument.c_str());
        }
        if (command == "dt")  {
            got_command = true;
            GLOBAL_timestep = atof(argument.c_str());
        }
        if (command == "iterations")  {
            got_command = true;
            GLOBAL_iterations = atof(argument.c_str());
        }
        if (command == "T_max")  {
            got_command = true;
            GLOBAL_max_simulation_time = atof(argument.c_str());
        }
        if (command == "save_each")  {
            got_command = true;
            GLOBAL_save_each = atoi(argument.c_str());
        }
        if (command == "snapshot_each")  {
            got_command = true;
            GLOBAL_snapshot_each = atoi(argument.c_str());
        }
        if (command == "springs")  {
            got_command = true;
            file_springs = argument;
        }
        if (command == "density")  {
            got_command = true;
            GLOBAL_density = atof(argument.c_str());
        }
        if (command == "friction")  {
            got_command = true;
            GLOBAL_friction = atof(argument.c_str());
        }
		if (command == "rolling_friction") {
			got_command = true;
			GLOBAL_rolling_friction = atof(argument.c_str());
		}
		if (command == "spinning_friction") {
			got_command = true;
			GLOBAL_spinning_friction = atof(argument.c_str());
		}
		if (command == "damping") {
			got_command = true;
			GLOBAL_damping = atof(argument.c_str());
		}
        if (command == "compliance")  {
            got_command = true;
            GLOBAL_compliance = atof(argument.c_str());
        }
		if (command == "rolling_compliance") {
			got_command = true;
			GLOBAL_rolling_compliance = atof(argument.c_str());
		}
		if (command == "spinning_compliance") {
			got_command = true;
			GLOBAL_spinning_compliance = atof(argument.c_str());
		}
        if (command == "penetrationrecovery")  {
            got_command = true;
            GLOBAL_penetrationrecovery = atof(argument.c_str());
        }
		if (command == "warmstart") {
			got_command = true;
			GLOBAL_warmstart = atoi(argument.c_str());
		}
		if (command == "SOR_solver") {
			got_command = true;
			use_SOR_solver = (bool)atoi(argument.c_str());
		}
        if (!got_command) {
            GetLog() << "ERROR. Unknown command in input line: " << command << "\n";
            return 0;
        } 
        iarg +=2;
    }

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Here set the inward-outward margins for collision shapes:
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    collision::ChCollisionSystemBullet::SetContactBreakingThreshold(0.01);

    //
    // HERE YOU POPULATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // The default material for the bricks:
    std::shared_ptr<ChMaterialSurfaceNSC> mmaterial(new ChMaterialSurfaceNSC);
    mmaterial->SetFriction(GLOBAL_friction); 
    //mmaterial->SetRestitution(0.0f); // either restitution, or compliance&damping, or none, but not both
    mmaterial->SetCompliance(GLOBAL_compliance);
    mmaterial->SetComplianceT(GLOBAL_compliance);
    mmaterial->SetDampingF(GLOBAL_damping);
	mmaterial->SetRollingFriction(GLOBAL_rolling_friction);
	mmaterial->SetSpinningFriction(GLOBAL_spinning_friction);
	mmaterial->SetComplianceRolling(GLOBAL_rolling_compliance);
	mmaterial->SetComplianceSpinning(GLOBAL_spinning_compliance);

    // Create the motion functions, if any
    if (file_motion_x != "") {
        load_motion(GLOBAL_motion_X, file_motion_x.c_str(), 0, GLOBAL_motion_amplifier, GLOBAL_motion_timestep);
        GLOBAL_use_motions = true;
    }
    if (file_motion_y != "") {
        load_motion(GLOBAL_motion_Y, file_motion_y.c_str(), 0, GLOBAL_motion_amplifier, GLOBAL_motion_timestep);
        GLOBAL_use_motions = true;
    }
    if (file_motion_z != "") {
        load_motion(GLOBAL_motion_Z, file_motion_z.c_str(), 0, GLOBAL_motion_amplifier, GLOBAL_motion_timestep);
        GLOBAL_use_motions = true;
    }

    std::unordered_map<int, std::shared_ptr<ChBody>> my_body_map;

    // Create all the rigid bodies loading their shapes from disk
    try {
        load_brick_file (mphysicalSystem, filename, mmaterial, my_body_map, (file_contacts == ""));
    }
    catch (ChException my_load_error) {
        GetLog()<< my_load_error.what();
        system("pause");
    }

	

    // Create all the springs loading from disk
    if (file_springs != "")
        try {
            load_spring_file (mphysicalSystem, file_springs, my_body_map);
        }
        catch (ChException my_load_error) {
            GetLog()<< my_load_error.what();
            system("pause");
        }

	// Create all the precomputed contacts from disk
	std::vector<PrecomputedContact> precomputed_contacts;
	std::shared_ptr<ChContactContainerNSC> precomputed_contact_container;

	if (file_contacts != "")
			try {
			load_contacts_file(mphysicalSystem, file_contacts, my_body_map, precomputed_contacts);
			precomputed_contact_container = std::make_shared<ChContactContainerNSC>();
			mphysicalSystem.Add(precomputed_contact_container);
		}
		catch (ChException my_load_error) {
			GetLog() << my_load_error.what();
			system("pause");
		}


    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Bricks test", core::dimension2d<u32>(960, 720), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(70.f, 120.f, -90.f),
                                    core::vector3df(30.f, 80.f, 160.f), 290, 190);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 1.6, 10), core::vector3df(0, 1.6, -3));
	
	application.SetSymbolscale(5e-5);
	application.SetContactsDrawMode(ChIrrTools::CONTACT_FORCES);
	
	// Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application.AssetUpdateAll();


    // Set no gravity on Y:
    application.GetSystem()->Set_G_acc(ChVector<>(0,-9.8,0));
    //application.GetSystem()->Set_G_acc(ChVector<>(9.8*0.2,-9.8,0));

    // Prepare the physical system for the simulation

	if (use_SOR_solver)
		mphysicalSystem.SetSolverType(ChSolver::Type::SYMMSOR);  // less precise, faster
	else
		mphysicalSystem.SetSolverType(ChSolver::Type::BARZILAIBORWEIN); // precise, slower

    mphysicalSystem.SetMaxPenetrationRecoverySpeed(GLOBAL_penetrationrecovery); 
    mphysicalSystem.SetMaxItersSolverSpeed(GLOBAL_iterations);
    mphysicalSystem.SetSolverWarmStarting(GLOBAL_warmstart);

    //
    // THE SOFT-REAL-TIME CYCLE
    //


    application.SetTimestep(GLOBAL_timestep);
    //application.SetPaused(true);

    if (GLOBAL_snapshot_each >0) {
        application.SetVideoframeSave(true);
        application.SetVideoframeSaveInterval(GLOBAL_snapshot_each);
    }

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 10, 10,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2, VECT_X)),
                             video::SColor(50, 90, 90, 150), true);

		// Populate precomputed contacts  if a file of contacts at t=0 is provided.
		if (precomputed_contact_container) {
			precomputed_contact_container->BeginAddContact();
			for (int i = 0; i < precomputed_contacts.size(); ++i) {
				precomputed_contact_container->AddContact( precomputed_contacts[i].GetCollisionInfo() );
			}
			precomputed_contact_container->EndAddContact();
		}
		ChIrrTools::drawAllContactPoints(precomputed_contact_container, application.GetVideoDriver(), 0.2, ChIrrTools::CONTACT_NORMALS);
		/*
		application.GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
		irr::video::SMaterial mattransp;
		mattransp.ZBuffer = false;
		mattransp.Lighting = false;
		application.GetVideoDriver()->setMaterial(mattransp);
		for (auto precompocont : precomputed_contacts) {
			irr::video::SColor mcol(200, 250, 250, 0);  // yellow vectors
			ChVector<> v1abs = precompocont.pos_t0;
			ChVector<> v2abs = precompocont.pos_t0 + precompocont.normal_t0;
			application.GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(v1abs), irr::core::vector3dfCH(v2abs), mcol);
		}
		*/
        application.DoStep();


        // Do some output to disk, for later postprocessing
        if (GLOBAL_save_each && (mphysicalSystem.GetStepcount() % GLOBAL_save_each  == 0))
        {
            // a) Use the contact callback object to save contacts:
            char contactfilename[200];
            sprintf(contactfilename, "%s%05d%s", "contacts", mphysicalSystem.GetStepcount(), ".txt");  // ex: contacts00020.tx
            _contact_reporter_class my_contact_rep;
            ChStreamOutAsciiFile result_contacts(contactfilename);
            my_contact_rep.mfile = &result_contacts;
            mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_rep);
			if (precomputed_contact_container)
				precomputed_contact_container->ReportAllContacts(&my_contact_rep);

            // b) Save rigid body positions and rotations
            char bodyfilename[200];
            sprintf(bodyfilename, "%s%05d%s", "bodies", mphysicalSystem.GetStepcount(), ".txt");  // ex: bodies00020.tx
            ChStreamOutAsciiFile result_bodies(bodyfilename);
			auto mbodies = mphysicalSystem.Get_bodylist().begin(); 
            while (mbodies != mphysicalSystem.Get_bodylist().end()) {
                result_bodies   << (*mbodies)->GetIdentifier()  << ", " 
                                << (*mbodies)->GetPos().x()  << ", "
                                << (*mbodies)->GetPos().y()  << ", "
                                << (*mbodies)->GetPos().z()  << ", "
                                << (*mbodies)->GetRot().e0()  << ", "
                                << (*mbodies)->GetRot().e1()  << ", "
                                << (*mbodies)->GetRot().e2()  << ", "
                                << (*mbodies)->GetRot().e3()  << ", "
                                << (*mbodies)->GetRotAngle() * CH_C_RAD_TO_DEG  << "\n";
                ++mbodies;
            }

            // b) Save spring reactions
            char springfilename[200];
            sprintf(springfilename, "%s%05d%s", "springs", mphysicalSystem.GetStepcount(), ".txt");  // ex: springs00020.tx
            ChStreamOutAsciiFile result_springs(springfilename);
            auto mlink = mphysicalSystem.Get_linklist().begin();
            while (mlink != mphysicalSystem.Get_linklist().end()) {
                if (auto mspring = std::dynamic_pointer_cast<ChLinkSpring>((*mlink)))
                result_springs  << mspring->GetIdentifier()  << ", " 
                                << mspring->Get_SpringReact()  << ", "
                                << mspring->GetMarker1()->GetAbsCoord().pos.x() << ", "
                                << mspring->GetMarker1()->GetAbsCoord().pos.y() << ", "
                                << mspring->GetMarker1()->GetAbsCoord().pos.z() << ", "
                                << mspring->GetMarker2()->GetAbsCoord().pos.x() << ", "
                                << mspring->GetMarker2()->GetAbsCoord().pos.y() << ", "
                                << mspring->GetMarker2()->GetAbsCoord().pos.z() << ", "
                                << "\n";
                ++mlink;
            }
        }

        // Force the simulator to close after N seconds
        if (application.GetSystem()->GetChTime() > GLOBAL_max_simulation_time)
            application.GetDevice()->closeDevice();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
