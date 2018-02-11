#ifndef OPENTISSUE_DYNAMICS_EDM_IO_EDM_SYSTEM_XML_READ_H
#define OPENTISSUE_DYNAMICS_EDM_IO_EDM_SYSTEM_XML_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm.h>

#include <TinyXML/tinyxml.h>

#include <string>
#include <fstream>
#include <sstream>


namespace OpenTissue
{

  namespace edm
  {

    namespace xml
    {

      // 2007-02-17 KE: I moved all thise streaming operators from being vector3 members into global operators in this edm namespace!!! They do not belong in the vector3 class!!!
      // 2009-03-11 kenny: Oh dear hardwired Vector3 type, creates a strong type dependency to another sub-library of OT
      template<typename T>
      inline Vector3<T> & operator<<(Vector3<T> & to, std::string const & from)
      {
        std::istringstream ist(from);
        char dummy;
        ist >> dummy >> to(0) >> dummy >> to(1) >> dummy >> to(2) >> dummy;
        return to;
      }

      template<typename T>
      inline Vector3<T> & operator<<(Vector3<T> & to,Vector3<T> const &from)
      {
        to = from;
        return to;
      }

      // 2009-03-11 kenny: Oh dear hardwired Vector3 type, creates a strong type dependency to another sub-library of OT
      template<typename T>
      inline Vector3<T> const & operator>>(Vector3<T> const & from, std::string & to)
      {
        std::ostringstream ost;
        ost << "[" << from(0) << "," << from(1) << "," << from(2) << "]" << std::endl;
        to = ost.str();
        return from;
      }

      // 2009-03-11 kenny: Oh dear hardwired Vector3 type, creates a strong type dependency to another sub-library of OT
      template<typename T>
      inline Vector3<T> const & operator>>(Vector3<T> const & from,Vector3<T> & to)
      {
        to = from;
        return from;
      }

      inline bool error(std::string const &err)
      {
        std::cerr << "EDM ERROR: " << err << std::endl;
        return false;
      }

      template<typename real_type>
      inline bool get_value_real(real_type& val, TiXmlElement const & elem, std::string const & att)
      {
        char const * attr = elem.Attribute(att);
        if (!attr) return false;
        std::istringstream ist(attr);
        ist >> val;
        return true;
      }

      template<typename tensor2_type>
      inline bool get_value_tensor2(tensor2_type& val, TiXmlElement const & elem, std::string const & att)
      {
        char const * attr = elem.Attribute(att);
        if (!attr) 
          return false;
        std::istringstream ist(attr);
        typename tensor2_type::value_type tmp;
        char dummy;
        ist >> dummy >> dummy >> tmp;
        val.t0[0] = tmp;
        ist >> dummy >> tmp;
        val.t0[1] = tmp;
        ist >> dummy >> dummy >> dummy >> tmp;
        val.t1[0] = tmp;
        ist >> dummy >> tmp;
        val.t1[1] = tmp;
        return true;
      }

      template<typename tensor3_type>
      inline bool get_value_tensor3(tensor3_type& val, TiXmlElement const & elem, std::string const & att)
      {
        const char* attr = elem.Attribute(att);
        if (!attr) return false;
        std::istringstream ist(attr);
        typename tensor3_type::value_type tmp;
        char dummy;
        ist >> dummy >> dummy >> tmp;
        val.t0[0] = tmp;
        ist >> dummy >> tmp;
        val.t0[1] = tmp;
        ist >> dummy >> tmp;
        val.t0[2] = tmp;
        ist >> dummy >> dummy >> dummy >> tmp;
        val.t1[0] = tmp;
        ist >> dummy >> tmp;
        val.t1[1] = tmp;
        ist >> dummy >> tmp;
        val.t1[2] = tmp;
        ist >> dummy >> dummy >> dummy >> tmp;
        val.t2[0] = tmp;
        ist >> dummy >> tmp;
        val.t2[1] = tmp;
        ist >> dummy >> tmp;
        val.t2[2] = tmp;
        return true;
      }

      inline bool get_value_compare(bool& val, TiXmlElement const & elem, std::string const & att, std::string const & cmp)
      {
        char const * attr = elem.Attribute(att);
        if (!attr) return false;
        val = cmp == attr;
        return true;
      }

      // HACK: the old index format was index="[n]" the new one is just index="n"
      // To avoid changing all data files, we simple support both formats ;)
      template<typename ulong_type>
      inline void get_correct_index(ulong_type& idx, std::istringstream& ist)
      {
        char tst = static_cast<char>(ist.peek());
        if ('[' == tst)
          ist >> tst;
        ist >> idx;
      }

      template<typename edm_system_type>
      inline bool create_surface(edm_system_type& system, TiXmlNode const & surface_node, std::string const & type, std::string const & id)
      {
        typedef typename edm_system_type::EDM_TYPES  edm_types;
        typedef Surface<edm_types>                   surface_type;
        typedef GenericBezierPatch<edm_types>        generic_bezier_patch_type;
        typedef QuadraticBezierPatch<edm_types>      quadratic_bezier_patch_type;
        typedef EllipsoidPatch<edm_types>            ellipsoid_patch_type;
        typedef typename Surface::SurfaceParticle    SurfaceParticle;

        const TiXmlElement* properties = surface_node.FirstChildElement("Properties");

        if (!properties)
          return error("'Body' force requieres a 'Properties' child node!");

        const char* timestep = properties->Attribute("timestep");
        if (!timestep)
          return error("A 'Body' object's 'Properties' node must have the delta t 'timestep' attribute!");
        typename Surface::real_type dt;
        std::istringstream ist1(timestep);
        ist1 >> dt;

        bool wrap_m = false;
        const char* mcont = properties->Attribute("mcont");
        if (mcont)
          wrap_m = std::string("Yes") == mcont;

        bool wrap_n = false;
        const char* ncont = properties->Attribute("ncont");
        if (ncont)
          wrap_n = std::string("Yes") == ncont;

        const TiXmlNode* rest = surface_node.FirstChild("NaturalShape");
        if (!(rest && rest == surface_node.LastChild("NaturalShape")))
          return error("One and exactly one 'NaturalShape' node must exist within a 'Body' node!");

        const TiXmlNode* InitialShapes = surface_node.FirstChild("InitialShape");
        if (InitialShapes && InitialShapes != surface_node.LastChild("InitialShape"))
          return error("Only one 'InitialShape' node can exist within a 'Body' node!");

        const TiXmlNode* particles = surface_node.FirstChild("Particles");
        if (!(particles && particles == surface_node.LastChild("Particles")))
          return error("One and exactly one 'Particles' node must exist within a 'Body' node!");

        // create a Generic Bezier Surface Patch (of user controlled order)
        if ("GenericBezierPatch"  == type) 
        {
          const char* Order = properties->Attribute("order");
          if (!Order)
            return error("The 'Properties' node of a concrete 'GenericBezierPatch' model must have the 'order' attribute!");
          size_t order;
          std::istringstream ist(Order);
          ist >> order;
          system.template create_model< generic_bezier_patch_type >(id)->set(order);
        }
        
        // create a Quadratic Bezier Surface Patch (3x3 control points)
        else if ("QuadraticBezierPatch"  == type)
          system.template create_model< quadratic_bezier_patch_type >(id);
        // create a Ellipsoid Surface Patch
        else if ("Ellipsoid"  == type)
          system.template create_model< ellipsoid_patch_type >(id);

        // surface not available
        else error("Deformable Surface type '"+type+"' not supported!");

        surface_type * surface = static_cast<surface_type*>(system.models().find(id)->second);

        // create natural shape
        {
          const TiXmlNode* Nodes = rest->FirstChild("Node");
          size_t n = 0;
          for (; Nodes; Nodes = Nodes->NextSibling("Node"), ++n) 
          {
            const TiXmlElement& Node = *Nodes->ToElement();
            const char* index = Node.Attribute("index");
            if (!index)
              return error("All 'NaturalShape' child nodes must have the 'index' attribute!");
            std::istringstream ist(index);
            size_t idx;
            get_correct_index(idx, ist);
            if (idx >= surface->nodes()) 
            {
              std::ostringstream ost;
              ost << "Index " << idx << " outta bounds for 'Node' no. " << n+1 << ".\n" <<
                "Index range for this Surface is [0.." << surface->nodes()-1 <<"]";
              return error(ost.str());
            }
            // get position
            const char* pos = Node.Attribute("pos");
            if (pos) 
            {
              typename surface_type::vector3_type r;
              r << pos;
              surface->set_natural_position(idx,r);
              if (!InitialShapes)
                surface->set_initial_position(idx,r);
            }
          }
        }

        // create initial shape
        if (InitialShapes) 
        {
          const TiXmlElement& init = *InitialShapes->ToElement();
          size_t n = 0;
          const TiXmlNode* Nodes = init.FirstChild("Node");
          for (; Nodes; Nodes = Nodes->NextSibling("Node"), ++n) 
          {
            const TiXmlElement& Node = *Nodes->ToElement();
            const char* index = Node.Attribute("index");
            if (!index)
              return error("All 'InitialShape' child nodes must have the 'index' attribute!");
            std::istringstream ist(index);
            size_t idx;
            get_correct_index(idx, ist);
            if (idx >= surface->nodes()) {
              std::ostringstream ost;
              ost << "Index " << idx << " outta bounds for 'Node' no. " << n+1 << ".\n" <<
                "Index range for this Surface is [0.." << surface->nodes()-1 <<"]";
              return error(ost.str());
            }
            // get position
            const char* pos = Node.Attribute("pos");
            if (pos) {
              typename surface_type::vector3_type r;
              r << pos;
              surface->set_initial_position(idx,r);
            }
          }
        }

        // create the particles
        char const * grid = particles->ToElement()->Attribute("grid");
        if (!grid)
          return error("A 'Body' object's 'Particles' node must have the 'grid' attribute!");
        std::istringstream ist(grid);
        size_t M,N;
        char dummy;
        ist >> dummy >> M >> dummy >> N >> dummy;

        surface->wrapping(wrap_m,wrap_n);
        if (!surface->initialize(M,N))
          return error("Deformable Surface "+type+" failed the initialization!");

        // proceed with particle properties
        const TiXmlElement* all = particles->FirstChildElement("All");
        if (all) // set All properties first 
        {  
          SurfaceParticle const M_default;  // sets all properties to their initial values!
          
          typename surface_type::real_type tmp1;
          bool tmp2;
          typename surface_type::tensor2_type tmp3;
          
          typename surface_type::real_type const mass     = get_value_real(tmp1, *all, "mass")            ? tmp1 : M_default.m;
          typename surface_type::real_type const damping  = get_value_real(tmp1, *all, "damping")         ? tmp1 : M_default.g;
          typename surface_type::tensor2_type   const tension  = get_value_tensor2(tmp3, *all, "tension")      ? tmp3 : M_default.e;
          typename surface_type::tensor2_type   const rigidity = get_value_tensor2(tmp3, *all, "rigidity")     ? tmp3 : M_default.x;
          bool                             const fixed    = get_value_compare(tmp2, *all, "fixed", "Yes") ? tmp2 : M_default.f;
          
          for (size_t n = 0; n < N; ++n)
            for (size_t m = 0; m < M; ++m)
              surface->set_mass(m,n,mass).set_damping(m,n,damping).set_tension(m,n,tension).set_rigidity(m,n,rigidity).set_fixed(m,n,fixed);
        }
        // set Single properties next
        const TiXmlNode* Singles = particles->FirstChild("Single");
        for (; Singles; Singles = Singles->NextSibling("Single")) 
        {
          const TiXmlElement& Single = *Singles->ToElement();
          const char* par = Single.Attribute("particle");
          if (!par)
            return error("A 'Single' child node must have the 'particle' attribute!");
          std::istringstream ist2(par);
          size_t m, n;
          char dummy2;
          ist2 >> dummy2 >> m >> dummy2 >> n >> dummy2;

          if (m>=M||n>=N) 
          {
            std::ostringstream ost;
            ost << "Particle [" << m << "," << n << "] is outta bounds in 'Single' node";
            return error(ost.str());
          }

          typename surface_type::real_type tmp1;
          bool tmp2; 
          typename surface_type::tensor2_type tmp3;
          
          if (get_value_real(tmp1, Single, "mass"))
            surface->set_mass(m,n,tmp1);
          if (get_value_real(tmp1, Single, "damping"))
            surface->set_damping(m,n,tmp1);
          if (get_value_tensor2(tmp3, Single, "tension"))
            surface->set_tension(m,n,tmp3);
          if (get_value_tensor2(tmp3, Single, "rigidity"))
            surface->set_rigidity(m,n,tmp3);
          if (get_value_compare(tmp2, Single, "fixed", "Yes"))
            surface->set_fixed(m,n,tmp2);
        }

        surface->timestep() = dt;

        return true;
      }

      template <typename edm_system_type>
      inline bool create_solid(edm_system_type& system, TiXmlNode const & solid_node, std::string const & type, std::string const & id)
      {
        typedef typename edm_system_type::EDM_TYPES  edm_types;
        typedef Solid<edm_types>                     solid_type;
        typedef GenericBezierSolid<edm_types>        generic_bezier_solid_type;
        typedef LinearBezierSolid<edm_types>         linear_bezier_solid_type;
        typedef EllipsoidSolid<edm_types>            ellipsoid_solid_type;
        typedef typename Solid::SolidParticle        SolidParticle;

        const TiXmlElement* properties = solid_node.FirstChildElement("Properties");

        if (!properties)
          return error("'Body' force requieres a 'Properties' child node!");

        const char* timestep = properties->Attribute("timestep");
        if (!timestep)
          return error("A 'Body' object's 'Properties' node must have the delta t 'timestep' attribute!");
        typename Solid::real_type dt;
        std::istringstream ist1(timestep);
        ist1 >> dt;

        bool wrap_l = false;
        const char* lcont = properties->Attribute("lcont");
        if (lcont)
          wrap_l = std::string("Yes") == lcont;

        bool wrap_m = false;
        const char* mcont = properties->Attribute("mcont");
        if (mcont)
          wrap_m = std::string("Yes") == mcont;

        bool wrap_n = false;
        const char* ncont = properties->Attribute("ncont");
        if (ncont)
          wrap_n = std::string("Yes") == ncont;

        const TiXmlNode* rest = solid_node.FirstChild("NaturalShape");
        if (!(rest && rest == solid_node.LastChild("NaturalShape")))
          return error("One and exactly one 'NaturalShape' node must exist within a 'Body' node!");

        const TiXmlNode* InitialShapes = solid_node.FirstChild("InitialShape");
        if (InitialShapes && InitialShapes != solid_node.LastChild("InitialShape"))
          return error("Only one 'InitialShape' node can exist within a 'Body' node!");

        const TiXmlNode* particles = solid_node.FirstChild("Particles");
        if (!(particles && particles == solid_node.LastChild("Particles")))
          return error("One and exactly one 'Particles' node must exist within a 'Body' node!");

        // create a Generic Bezier Solid Patch (of user controlled order)
        if ("GenericBezierSolid"  == type) {
          const char* Order = properties->Attribute("order");
          if (!Order)
            return error("The 'Properties' node of a concrete 'GenericBezierSolid' model must have the 'order' attribute!");
          size_t order;
          std::istringstream ist(Order);
          ist >> order;
          system.template create_model<generic_bezier_solid_type>(id)->set(order);
        }
        // create a Linear Bezier Solid (2x2x2 control points)
        else if ("LinearBezierSolid"  == type)
          system.template create_model<linear_bezier_solid_type>(id);
        // create a Ellipsoid Solid
        else if ("Ellipsoid"  == type)
          system.template create_model<ellipsoid_solid_type>(id);

        // surface not available
        else return error("Deformable Solid type '"+type+" not supported!");

        solid_type* solid = static_cast<solid_type*>(system.models().find(id)->second);

        // create natural shape
        {
          const TiXmlNode* Nodes = rest->FirstChild("Node");
          size_t n = 0;
          for (; Nodes; Nodes = Nodes->NextSibling("Node"), ++n)
          {
            const TiXmlElement& Node = *Nodes->ToElement();
            const char* index = Node.Attribute("index");
            if (!index)
              return error("All 'NaturalShape' child nodes must have the 'index' attribute!");
            std::istringstream ist(index);
            size_t idx;
            get_correct_index(idx, ist);
            if (idx >= solid->nodes()) 
            {
              std::ostringstream ost;
              ost << "Index " << idx << " outta bounds for 'Node' no. " << n+1 << ". " <<
                "Index range for this Solid is [0.." << solid->nodes()-1 <<"]!";
              return error(ost.str());
            }
            // get position
            char const * pos = Node.Attribute("pos");
            if (pos) 
            {
              typename Solid::vector3_type r;
              r << pos;
              solid->set_natural_position(idx,r);
              if (!InitialShapes)
                solid->set_initial_position(idx,r);
            }
          }
        }

        // create initial shape
        if (InitialShapes) 
        {
          TiXmlElement const & init = *InitialShapes->ToElement();
          size_t n = 0;
          const TiXmlNode* Nodes = init.FirstChild("Node");
          for (; Nodes; Nodes = Nodes->NextSibling("Node"), ++n) 
          {
            TiXmlElement const & Node = *Nodes->ToElement();
            char const * index = Node.Attribute("index");
            if (!index)
              return error("All 'InitialShape' child nodes must have the 'index' attribute!");
            std::istringstream ist(index);
            size_t idx;
            get_correct_index(idx, ist);
            if (idx >= solid->nodes()) 
            {
              std::ostringstream ost;
              ost << "Index " << idx << " outta bounds for 'Node' no. " << n+1 << ". " <<
                "Index range for this Solid is [0.." << solid->nodes()-1 <<"]!";
              return error(ost.str());
            }
            // get position
            char const * pos = Node.Attribute("pos");
            if (pos) 
            {
              typename solid_type::vector3_type r;
              r << pos;
              solid->set_initial_position(idx,r);
            }
          }
        }

        // create the particles
        char const * grid = particles->ToElement()->Attribute("grid");
        if (!grid)
          return error("A 'Body' object's 'Particles' node must have the 'grid' attribute!");
        std::istringstream ist(grid);
        size_t L,M,N;
        char dummy;
        ist >> dummy >> L >> dummy >> M >> dummy >> N >> dummy;

        solid->wrapping(wrap_l,wrap_m,wrap_n);
        if (!solid->initialize(L,M,N))
          return error("Deformable Solid "+type+" failed the initialization!");

        // proceed with particle properties
        TiXmlElement const * all = particles->FirstChildElement("All");
        if (all) // set All properties first 
        {  
          SolidParticle const M_default;  // sets all properties to their initial values!
          
          typename solid_type::real_type tmp1;
          bool tmp2;
          typename solid_type::tensor3_type tmp3; typename solid_type::tensor2_type tmp4;
          
          typename solid_type::real_type const mass    = get_value_real(tmp1, *all, "mass")            ? tmp1 : M_default.m;
          typename solid_type::real_type const damping = get_value_real(tmp1, *all, "damping")         ? tmp1 : M_default.g;
          typename solid_type::tensor2_type   const spatial = get_value_tensor2(tmp4, *all, "spatial")      ? tmp4 : M_default.u;
          typename solid_type::tensor3_type   const tension = get_value_tensor3(tmp3, *all, "tension")      ? tmp3 : M_default.e;
          bool                           const fixed   = get_value_compare(tmp2, *all, "fixed", "Yes") ? tmp2 : M_default.f;
          
          for (size_t n = 0; n < N; ++n)
            for (size_t m = 0; m < M; ++m)
              for (size_t l = 0; l < L; ++l)
                 solid->set_mass(l,m,n,mass).set_damping(l,m,n,damping).set_tension(l,m,n,tension).set_spatial(l,m,n,spatial).set_fixed(l,m,n,fixed);
        }
        // set Single properties first
        TiXmlNode const * Singles = particles->FirstChild("Single");
        for (; Singles; Singles = Singles->NextSibling("Single")) 
        {
          TiXmlElement const & Single = *Singles->ToElement();
          char const * par = Single.Attribute("particle");
          if (!par)
            return error("A 'Single' child node must have the 'particle' attribute!");
          std::istringstream ist2(par);
          size_t l, m, n;
          char dummy2;
          ist2 >> dummy2 >> l >> dummy2 >> m >> dummy2 >> n >> dummy2;

          if (l>=L||m>=M||n>=N) 
          {
            std::ostringstream ost;
            ost << "Particle [" << l << "," << m << "," << n << "] is outta bounds in 'Single' node";
            return error(ost.str());
          }

          typename solid_type::real_type tmp1;
          bool tmp2;
          typename solid_type::tensor3_type tmp3;
          typename solid_type::tensor2_type tmp4;
          
          if (get_value_real(tmp1, Single, "mass"))
            solid->set_mass(l,m,n,tmp1);
          if (get_value_real(tmp1, Single, "damping"))
            solid->set_damping(l,m,n,tmp1);
          if (get_value_tensor3(tmp3, Single, "tension"))
            solid->set_tension(l,m,n,tmp3);
          if (get_value_tensor2(tmp4, Single, "spatial"))
            solid->set_spatial(l,m,n,tmp4);
          if (get_value_compare(tmp2, Single, "fixed", "Yes"))
            solid->set_fixed(l,m,n,tmp2);
        }

        solid->timestep() = dt;

        return true;
      }

      template <typename edm_system_type>
      inline bool create_models(edm_system_type& system, TiXmlNode const & models_node)
      {
        typedef typename edm_system_type::EDM_TYPES  edm_types;
        typedef typename edm_types::model_type       model_type;
        typedef          Surface<edm_types>          surface_type;
        typedef          Solid<edm_types>            solid_type;
        
        typedef typename edm_system_type::EDMIOMaterials  EDMIOMaterials;
        typedef typename edm_system_type::EDMIOForces     EDMIOForces;
        typedef typename edm_system_type::EDMIOObjects    EDMIOObjects;
        
        EDMIOMaterials const & materials = system.materials();
        EDMIOForces    const & forces    = system.forces();
        EDMIOObjects   const & objects   = system.objects();

        TiXmlNode const * Bodies = models_node.FirstChild("Body");
        for (; Bodies; Bodies = Bodies->NextSibling("Body")) 
        {
          TiXmlElement const & body = *Bodies->ToElement();
          std::string val;
          // retrieve all mandatory/optional attributes
          char const * id = body.Attribute("id");
          if (!id)
            return error("'Body' node must have an 'id' attribute!");
          val = id;
          if (system.models().find(val) != system.models().end())
            return error("Deformable Body with id = '"+val+"' already exist!");

          char const * type = body.Attribute("type");
          if (!type)
            return error("'Body' node must have a 'type' attribute!");

          char const * model = body.Attribute("model");
          if (!model)
            return error("'Body' node must have a 'model' attribute!");
          val = model;

          // create deformable curve
          if ("Curve" == val)
            return error("A deformable body model of type '"+val+"' is not (yet) supported!");

          // create deformable surface
          else if ("Surface" == val) {
            if (!create_surface(system, body, type, id))
              return false;
          }

          // create deformable surface
          else if ("Solid" == val) {
            if (!create_solid(system, body, type, id))
              return false;
          }

          // model not available
          else return error("'model' attribute value '"+val+" not recognized in node 'Body'");


          model_type * deform = system.models().find(id)->second;

          // perform all bindings quick 'n dirty
          TiXmlNode* const  bindings = body.FirstChild("Bind");
          for (; bindings; bindings = bindings->NextSibling("Bind")) 
          {
            TiXmlElement const & bind = *bindings->ToElement();
            std::string val2;
            // retrieve all mandatory/optional attributes
            char const * type2 = bind.Attribute("type");
            if (!type2)
              return error("Mandatory attribute 'type' not found!");
            val2 = type2;
            if ("Collision" != val2 && "Force" != val2 && "Material" != val2)
              return error("'type' attribute value '"+val2+"' not recognized in node 'Bind'");

            char const * ref = bind.Attribute("ref");
            if (!ref)
              return error("Mandatory attribute 'ref' not found!");
            val2 = ref;
            typename EDMIOObjects::const_iterator   obj_itor   = objects.find(val2);
            typename EDMIOForces::const_iterator    force_itor = forces.find(val2);
            typename EDMIOMaterials::const_iterator mat_itor   = materials.find(val2);
            char const * par = bind.Attribute("particle");

            // perform the actual binding (all should be okay now (except for particle)
            if (std::string("Material") == type2)  // material binding
              if (mat_itor == materials.end())
                return error("Material with id = '"+val2+"' does not exist!");
              else
                deform->add(*mat_itor->second);
            else if (std::string("Collision") == type2)  // collision binding
              if (obj_itor == objects.end())
                return error("Rigid Object with id = '"+val2+"' does not exist!");
              else
                deform->add(*obj_itor->second);
            else  // force binding
              if (force_itor == forces.end())
                return error("External Force with id = '"+val2+"' does not exist!");
              else if (!par)
                deform->add(*force_itor->second);
              else {
                std::istringstream ist(par);
                size_t a1, a2, a3;
                char dummy;
                ist >> dummy >> a1 >> dummy >> a2 >> dummy >> a3 >> dummy;
                switch (deform->type()) 
                {
                  case edm_types::EDM_Surface:
                    deform->add(static_cast<surface_type*>(deform)->particle(a1,a2), *force_itor->second);
                    break;
                  case edm_types::EDM_Solid:
                    deform->add(static_cast<solid_type*>(deform)->particle(a1,a2,a3), *force_itor->second);
                    break;
                  }
              }
          }
        }

        return true;
      }

      template <typename edm_system>
      inline bool create_objects(edm_system & system, TiXmlNode const & objects_node)
      {
        typedef typename edm_system::edm_types::object_type  object_type;
        typedef EDMPlane<typename edm_system::edm_types>     EDMPlane;
        typedef EDMSphere<typename edm_system::edm_types>    EDMSphere;
        typedef EDMTorus<typename edm_system::edm_types>     EDMTorus;
        typedef EDMCylinder<typename edm_system::edm_types>  EDMCylinder;

        TiXmlNode const * Objects = objects_node.FirstChild("Object");
        for (; Objects; Objects = Objects->NextSibling("Object")) 
        {
          TiXmlElement const & object = *Objects->ToElement();
          std::string val;
          // retrieve all mandatory/optional attributes
          char const * id = object.Attribute("id");
          if (!id)
            return error("'Object' node must have an 'id' attribute!");
          val = id;
          if (system.objects().find(val) != system.objects().end())
            return error("Rigid Object with id = '"+val+"' already exist!");

          char const * type = object.Attribute("type");
          if (!type)
            return error("'Object' node must have an 'type' attribute!");
          val = type;

          TiXmlElement const * properties = object.FirstChildElement("Properties");

          // create a Plane object
          if ("Plane" == val) 
          {
            if (!properties)
              return error("'Plane' object requieres a 'Properties' child node!");
            char const * X0 = properties->Attribute("x0");
            if (!X0)
              return error("A 'Plane' object's 'Properties' node must have the fixed point 'x0' attribute!");
            val = X0;
            typename EDMPlane::vector3_type x0;
            x0 << val;

            char const * N = properties->Attribute("n");
            if (!N)
              return error("A 'Plane' object's 'Properties' node must have the plane normal 'n' attribute!");
            val = N;
            typename EDMPlane::vector3_type n;
            n << val;

            system.template create_object<EDMPlane>(id)->set(x0,n);
          }

          // create a Sphere object
          else if ("Sphere" == val) 
          {
            if (!properties)
              return error("'Sphere' object requieres a 'Properties' child node!");
            char const * R = properties->Attribute("r");
            if (!R)
              return error("A 'Sphere' object's 'Properties' node must have the radius 'r' attribute!");
            typename EDMSphere::real_type r;
            std::istringstream ist(R);
            ist >> r;

            char const * C = properties->Attribute("c");
            if (!C)
              return error("A 'Sphere' object's 'Properties' node must have the fixed center 'c' attribute!");
            val = C;
            typename EDMSphere::vector3_type c;
            c << val;

            system.template create_object<EDMSphere>(id)->set(r,c);
          }

          // create a Torus object
          else if ("Torus" == val) 
          {
            if (!properties)
              return error("'Torus' object requieres a 'Properties' child node!");

            char const * A = properties->Attribute("a");
            if (!A)
              return error("A 'Torus' object's 'Properties' node must have the torus tupe radius 'a' attribute!");
            typename EDMTorus::real_type a;
            std::istringstream ist1(A);
            ist1 >> a;

            char const * R = properties->Attribute("r");
            if (!R)
              return error("A 'Torus' object's 'Properties' node must have the hole radius 'r' attribute!");
            typename EDMTorus::real_type r;
            std::istringstream ist2(R);
            ist2 >> r;

            char const * C = properties->Attribute("c");
            if (!C)
              return error("A 'Torus' object's 'Properties' node must have the torus center 'c' attribute!");
            val = C;
            typename EDMTorus::vector3_type c;
            c << val;

            system.template create_object<EDMTorus>(id)->set(a,r,c);
          }

          // create a Cylinder object
          else if ("Cylinder" == val) 
          {
            if (!properties)
              return error("'Cylinder' object requieres a 'Properties' child node!");

            char const * R = properties->Attribute("r");
            if (!R)
              return error("A 'Cylinder' object's 'Properties' node must have the cylinder radius 'r' attribute!");
            typename EDMCylinder::real_type r;
            std::istringstream ist1(R);
            ist1 >> r;

            char const * H = properties->Attribute("h");
            if (!H)
              return error("A 'Cylinder' object's 'Properties' node must have the height (length) 'h' attribute!");
            typename EDMCylinder::real_type h;
            std::istringstream ist2(H);
            ist2 >> h;

            char const * C = properties->Attribute("c");
            if (!C)
              return error("A 'Cylinder' object's 'Properties' node must have the center 'c' attribute!");
            val = C;
            typename EDMCylinder::vector3_type c;
            c << val;

            system.template create_object<EDMCylinder>(id)->set(c,r,r,h);
          }

          // object not available
          else return error("'type' attribute value '"+val+"' not recognized in node 'Object'");


          object_type * obj = system.objects().find(id)->second;

          char const * visible = object.Attribute("visible");
          if (visible)
            obj->set_visibility(std::string("Yes") == visible);

          char const * E = object.Attribute("epsilon");
          if (E) 
          {
            std::istringstream ist(E);
            typename Object::real_type e;
            ist >> e;
            obj->set_epsilon(e);
          }

          char const * Scale = object.Attribute("scale");
          if (Scale) 
          {
            std::istringstream ist(Scale);
            typename Object::real_type scale;
            ist >> scale;
            obj->set_scale(scale);
          }

          // perform all material bindings quick 'n dirty
          TiXmlNode const * bindings = object.FirstChild("Bind");
          if (bindings)
          {  // as objects only support one material, no need to waste time for anything more!
            TiXmlElement const & bind = *bindings->ToElement();
            std::string val2;
            // retrieve all mandatory/optional attributes
            char const * type2 = bind.Attribute("type");
            if (!type2)
              return error("Mandatory attribute 'type' not found for 'Bind' node!");
            if (std::string("Material") != type2)
              return error("Only 'Material' bindings are supported for objects");

            // material binding
            char const * ref = bind.Attribute("ref");
            if (!ref)
              return error("Mandatory attribute 'ref' not found!");
            val2 = ref;
            typename edm_system_type::EDMIOMaterials::const_iterator mat_itor = system.materials().find(val2);
            if (mat_itor == system.materials().end())
              return error("Material with id = '"+val2+"' does not exist!");
            else
              obj->set_material(*mat_itor->second);
          }
        }

        return true;
      }

      template<typename edm_system>
      inline bool create_forces(edm_system & system, TiXmlNode const & forces_node)
      {
        typedef Gravity<typename edm_system::edm_types>  gravity_type;
        typedef Viscous<typename edm_system::edm_types>  viscous_type;
        typedef Spring<typename edm_system::edm_types>   spring_type;

        TiXmlNode const * Forces = forces_node.FirstChild("Force");
        for (; Forces; Forces = Forces->NextSibling("Force")) 
        {
          TiXmlElement const & force = *Forces->ToElement();
          std::string val;
          // retrieve all mandatory/optional attributes
          char const * id = force.Attribute("id");
          if (!id)
            return error("'Force' node must have an 'id' attribute!");
          val = id;
          if (system.forces().find(val) != system.forces().end())
            return error("External Force with id = '"+val+"' already exist!");

          char const * type = force.Attribute("type");
          if (!type)
            return error("'Force' node must have a 'type' attribute!");
          val = type;

          TiXmlElemen const t* properties = force.FirstChildElement("Properties");

          // create a Gravity force
          if ("Gravity" == val)
          {
            if (!properties)
              return error("'Gravity' force requieres a 'Properties' child node!");
            char const * G = properties->Attribute("g");
            if (!G)
              return error("A 'Gravity' force's 'Properties' node must have the gravitationsl field 'g' attribute!");
            val = G;
            typename gravity_type::vector3_type g;
            g << val;
            system.template create_force<gravity_type>(id)->set(g);
          }

          // create a Viscous force
          else if ("Viscous" == val) 
          {
            if (!properties)
              return error("'Viscous' force requieres a 'Properties' child node!");
            char const * C = properties->Attribute("c");
            if (!C)
              return error("A 'Viscous' force's 'Properties' node must have the fluid strength 'c' attribute!");
            typename viscous_type::real_type c;
            std::istringstream ist(C);
            ist >> c;
            char const * U = properties->Attribute("u");
            if (!U)
              return error("A 'Viscous' force's 'Properties' node must have the constant stream velocity 'u' attribute!");
            val = U;
            typename viscous_type::vector3_type u;
            u << val;
            system.template create_force<viscous_type>(id)->set(c,u);
          }

          // create a Spring force
          else if ("Spring" == val) 
          {
            if (!properties)
              return error("'Spring' force requieres a 'Properties' child node!");
            char const * K = properties->Attribute("k");
            if (!K)
              return error("A 'Spring' force's 'Properties' node must have the spring constant 'k' attribute!");
            typename Spring::real_type k;
            std::istringstream ist(K);
            ist >> k;
            char const * R0 = properties->Attribute("r0");
            if (!R0)
              return error("A 'Spring' force's 'Properties' node must have the fixed world point 'r0' attribute!");
            val = R0;
            typename Spring::vector3_type r0;
            r0 << val;
            system.template create_force<spring_type>(id)->set(k,r0);
          }

          // force not available
          else return error("'type' attribute value '"+val+"' not recognized in node 'Force'");
        }

        return true;
      }

      //template<typename edm_system_type>
      //bool create_materials(edm_system_type& system, const TiXmlNode& materials_node)
      //{
      //  typedef EDMColor<typename edm_system_type::EDM_TYPES>  EDMColor;
      //  typedef EDMTexture<typename edm_system_type::EDM_TYPES>  EDMTexture;
      //
      //  const TiXmlNode* Materials = materials_node.FirstChild("Material");
      //  for (; Materials; Materials = Materials->NextSibling("Material")) {
      //    const TiXmlElement& material = *Materials->ToElement();
      //    std::string val;
      //    // retrieve all mandatory/optional attributes
      //    const char* id = material.Attribute("id");
      //    if (!id)
      //      return error("'Material' node must have an 'id' attribute!");
      //    val = id;
      //    if (system.materials().find(val) != system.materials().end())
      //      return error("Material with id = '"+val+"' already exist!");
      //
      //    const char* type = material.Attribute("type");
      //    if (!type)
      //      return error("'Material' node must have an 'type' attribute!");
      //    val = type;
      //
      //    const TiXmlElement* properties = material.FirstChildElement("Properties");
      //
      //    // create a Color material
      //    if ("Color" == val) {
      //      if (!properties)
      //        return error("'Color' material requieres a 'Properties' child node!");
      //      const char* RGB = properties->Attribute("rgb");
      //      if (!RGB)
      //        return error("A 'Color' material's 'Properties' node must have the 'rgb' attribute!");
      //      typename EDMColor::real_type r,g,b;
      //      char dummy;
      //      std::istringstream ist(RGB);
      //      ist >> r >> dummy >> g >> dummy >> b;
      //      EDMColor* color = static_cast<EDMColor*>(system.template create_material<EDMColor>(id));
      //      if (!color->create(r,g,b)) {
      //        val = id;
      //        return error("'Color' material '"+val+"' failed the creation!");
      //      }
      //    }
      //
      //    // create a Texture material
      //    else if ("Texture" == val) {
      //      if (!properties)
      //        return error("'Texture' material requieres a 'Properties' child node!");
      //      const char* File = properties->Attribute("file");
      //      if (!File)
      //        return error("A 'Texture' material's 'Properties' node must have the texture filename 'file' attribute!");
      //      EDMTexture* texture = static_cast<EDMTexture*>(system.template create_material<EDMTexture>(id));
      //      if (!texture->create(File)) {
      //        val = id;
      //        return error("'Texture' material '"+val+"' failed the creation!");
      //      }
      //    }
      //
      //    // material not available
      //    else return error("'type' attribute value '"+val+"' not recognized in node 'Material'");
      //  }
      //
      //  return true;
      //}

      template<typename edm_system>
      inline bool create_system(edm_system & system, TiXmlNode const & root_node)
      {
        TiXmlNode const * EDMConfig = root_node.FirstChild("EDMConfig");
        if (!(EDMConfig && EDMConfig == root_node.LastChild("EDMConfig")))
          return error("One and exactly one 'EDMConfig' node must exist!");

        //// create all materials
        //const TiXmlNode* MaterialContainer = EDMConfig->FirstChild("MaterialContainer");
        //if (!(MaterialContainer && MaterialContainer == EDMConfig->LastChild("MaterialContainer")))
        //  return error("One and exactly one 'MaterialContainer' node must exist!");
        //if (!create_materials(system, *MaterialContainer))
        //  return false;

        // create all forces
        TiXmlNode const * ForceContainer = EDMConfig->FirstChild("ExternalForces");
        if (!(ForceContainer && ForceContainer == EDMConfig->LastChild("ExternalForces")))
          return error("One and exactly one 'ExternalForces' node must exist!");
        if (!create_forces(system, *ForceContainer))
          return false;

        // create all objects
        TiXmlNode const * ObjectContainer = EDMConfig->FirstChild("RigidObjects");
        if (!(ObjectContainer && ObjectContainer == EDMConfig->LastChild("RigidObjects")))
          return error("One and exactly one 'RigidObjects' node must exist!");
        if (!create_objects(system, *ObjectContainer))
          return false;

        // create all deformable models
        TiXmlNode const * ModelContainer = EDMConfig->FirstChild("DeformableBodies");
        if (!(ModelContainer && ModelContainer == EDMConfig->LastChild("DeformableBodies")))
          return error("One and exactly one 'DeformableBodies' node must exist!");
        if (!create_models(system, *ModelContainer))
          return false;

        return true;
      }

    }  // namespace xml


    template<typename edm_system>
    inline bool edm_system_xml_read(edm_system & system, std::string const & filename)
    {
      std::ifstream in(filename.c_str(), std::ios::binary);
      if (!in)
        return xml::error("file \"" + filename + "\" not found!");
      unsigned long len = in.seekg(0, std::ios::end).tellg();
      if (!len)
        return xml::error("file has no data!");
      char *data = new char[len+1];
      data[len]=0;
      in.seekg(0).read(data, len);
      in.close();
      std::string xml = data;
      delete[] data;

      TiXmlDocument doc;
      doc.Parse(xml.c_str());
      if (doc.Error())
        return xml::error(doc.ErrorDesc());
      if (!xml::create_system(system, doc))
        return false;

      return true;
    }

  } // namespace edm

} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_EDM_IO_EDM_SYSTEM_XML_READ_H
#endif
