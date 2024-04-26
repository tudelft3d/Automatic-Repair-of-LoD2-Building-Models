#pragma once
#ifndef BuildingPCCData_mesh_holefilling_hpp
#define BuildingPCCData_mesh_holefilling_hpp

#define COMPILER_MSVC
#define NOMINMAX

#include <vld.h> 
#include <omp.h>
#include <iostream>
#include <vector>
#include <map>
#include <numeric> // std::iota

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/surface_mesh_builder.h>
#include <easy3d/algo/delaunay_2d.h>
#include <easy3d/algo/surface_mesh_geometry.h>
#include <easy3d/algo_ext/surfacer.h>
#include <easy3d/kdtree/kdtree_search_eth.h>


#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/graph.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/viewer/viewer.h>

#include "additional_repair.hpp"
#include "mesh_properties.hpp"
#include "remove_inner_faces.hpp"

namespace building_pcc_data
{
	int easy3d_fill_holes(easy3d::SurfaceMesh*, const bool allow_internal_log_print = true);

	int cgal_hole_filling(easy3d::SurfaceMesh*, std::vector<easy3d::SurfaceMesh::Halfedge>&);

	void holes_filling_preprocessing(easy3d::SurfaceMesh*, std::map<easy3d::SurfaceMesh::Vertex, int>&);

	void holes_filling_postprocessing(easy3d::SurfaceMesh*);

	void complex_holes_detection(easy3d::SurfaceMesh*, std::map<easy3d::SurfaceMesh::Vertex, int>&, std::vector<std::vector<std::pair<int, int>>>&, int&, int&);

	void get_edge_neighbors
	(
		easy3d::SurfaceMesh*,
		std::vector<std::pair<int, int>>&,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		std::map<int, std::set<int>>&
	);

	void get_two_edge_attributes
	(
		easy3d::SurfaceMesh*,
		easy3d::SurfaceMesh::Vertex&, easy3d::SurfaceMesh::Vertex&, easy3d::SurfaceMesh::Vertex&,
		easy3d::SurfaceMesh::Vertex&, easy3d::SurfaceMesh::Vertex&, easy3d::SurfaceMesh::Vertex&, easy3d::SurfaceMesh::Vertex&,
		std::set<int>&,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		int&, int&, int&, int&,
		bool&
	);

	void get_two_edge_incident_facets
	(
		easy3d::SurfaceMesh*,
		std::set<int>&,
		std::set<int>&
	);

	void create_face_from_two_edges
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Vertex&,
		easy3d::SurfaceMesh::Vertex&,
		easy3d::SurfaceMesh::Vertex&,
		easy3d::vec3&, easy3d::vec3&, easy3d::vec3&
	);

	void get_all_pre_h_candidates
	(
		easy3d::SurfaceMesh*,
		easy3d::SurfaceMesh::Halfedge&,
		easy3d::SurfaceMesh::Halfedge&,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		std::map<std::pair<int, int>, int>&,
		std::set<int>&
	);

	void get_hole_candidates
	(
		std::map<std::pair<int, int>, bool>&,
		std::pair<int, int>&, std::pair<int, int>&, std::pair<int, int>&,
		std::pair<int, int>&, std::pair<int, int>&, std::pair<int, int>&,
		std::vector<std::pair<int, int>>&,
		std::map<int, int>&,
		int&, int&, int&, int&
	);

	void hole_detection_pipeline
	(
		easy3d::SurfaceMesh* ,
		easy3d::SurfaceMesh::Halfedge& ,
		easy3d::SurfaceMesh::Halfedge& ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		std::map<std::pair<int, int>, bool>& ,
		std::map<int, int>& ,
		std::vector<std::pair<int, int>>& 
	);

	void copy_mesh(easy3d::SurfaceMesh*, easy3d::SurfaceMeshBuilder&);

	void add_missing_edge
	(
		std::map< std::pair<int, int>, bool>& ,
		std::vector<std::pair<int, int>>& ,
		const int ,
		const int ,
		const easy3d::SurfaceMesh::Vertex ,
		const easy3d::SurfaceMesh::Vertex 
	);

	void check_vertices_for_missing_edges
	(
		easy3d::SurfaceMesh* ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		std::map< int, bool>& ,
		std::map< std::pair<int, int>, bool>& ,
		std::vector<std::pair<int, int>>& ,
		const easy3d::vec3 ,
		const int 
	);

	void find_missing_edges_in_holes
	(
		easy3d::SurfaceMesh* ,
		std::vector<std::pair<int, int>>& ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& 
	);


	bool planar_hole_checking
	(
		easy3d::SurfaceMesh* ,
		std::vector<std::pair<int, int>>& ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		easy3d::Plane3& ,
		std::map<int, bool>&
	);

	bool check_visited_turn
	(
		std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, bool>& ,
		std::pair<int, int>& ,
		std::pair<int, int>& ,
		std::pair<int, int>& ,
		std::pair<int, int>& 
	);

	bool check_hole_loop_completeness(std::vector<std::pair<int, int>>&);

	easy3d::SurfaceMesh::Vertex get_non_adjacent_vertex
	(
		easy3d::SurfaceMesh*,
		easy3d::SurfaceMesh::Vertex,
		easy3d::SurfaceMesh::Vertex,
		easy3d::SurfaceMesh::Vertex,
		easy3d::SurfaceMesh::Vertex,
		std::map<easy3d::SurfaceMesh::Vertex, int>&
	);

	void hole_edge_complement
	(
		easy3d::SurfaceMesh*,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		std::vector<std::vector<std::pair<int, int>>>&,
		std::vector<std::vector<std::pair<int, int>>>&
	);

	void filter_invalid_hole_edges
	(
		easy3d::SurfaceMesh*,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		std::vector<std::vector<std::pair<int, int>>>&,
		std::vector<std::vector<std::pair<int, int>>>&
	);

	void re_order_hole_vertices
	(
		easy3d::SurfaceMesh* ,
		std::vector<std::pair<int, int>>& ,
		std::vector<std::vector<std::pair<int, int>>>& ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		std::map<std::pair<int, int>, int>&,
		std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, bool>&
	);

	void get_hole_start_end_vertex
	(
		easy3d::SurfaceMesh* ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		easy3d::SurfaceMesh::Vertex& ,
		easy3d::SurfaceMesh::Vertex& ,
		easy3d::SurfaceMesh::Vertex& ,
		easy3d::SurfaceMesh::Vertex& ,
		std::map<int, bool>& ,
		const std::vector<std::pair<int, int>> 
	);

	void reverse_hole_edge_vertices(std::vector<std::pair<int, int>>&);

	bool hole_edge_collinear_check
	(
		easy3d::SurfaceMesh* ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		const std::vector<std::pair<int, int>>
	);

	void planar_hole_edge_complement
	(
		easy3d::SurfaceMesh* ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		std::map<int, bool>&,
		std::vector<std::pair<int, int>>&,
		easy3d::SurfaceMesh::Vertex ,
		easy3d::SurfaceMesh::Vertex ,
		const easy3d::SurfaceMesh::Vertex ,
		const easy3d::SurfaceMesh::Vertex ,
		const easy3d::Plane3 ,
		const bool 
	);

	void project_hole_points
	(
		easy3d::SurfaceMesh*,
		std::vector<std::pair<int, int>>&,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		std::vector<CDPoint_in>&,
		std::map<int, std::pair<easy3d::vec3, easy3d::vec2>>&
	);

	bool extract_polyline_from_holes
	(
		easy3d::SurfaceMesh*,
		std::vector<std::pair<int, int>>&,
		std::map<easy3d::SurfaceMesh::Vertex, int>&,
		Polygon_2_in&,
		std::vector< std::pair<CDPoint_in, unsigned> >&,
		std::map<int, int>&
	);

	void hole_surface_recosntruction_cgal_constrained_delaunay
	(
		easy3d::SurfaceMesh*,
		Polygon_2_in&,
		std::vector< std::pair<CDPoint_in, unsigned> >&,
		std::map<int, int>&,
		std::vector<Facet>&,
		const bool,
		const int
	);

	void triangulate_holes_without_self_intersection
	(
		easy3d::SurfaceMesh* ,
		std::map<easy3d::SurfaceMesh::Vertex, int>& ,
		easy3d::SurfaceMeshBuilder& ,
		std::vector<Facet>& 
	);

	void remesh_holes(easy3d::SurfaceMesh*, std::map<easy3d::SurfaceMesh::Vertex, int>&, std::vector< std::vector <std::pair<int, int>>>&, int&);

	void advacing_holes_filling(easy3d::SurfaceMesh*);
}

#endif