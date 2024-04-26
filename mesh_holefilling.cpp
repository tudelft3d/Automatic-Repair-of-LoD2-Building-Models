#include "mesh_holefilling.hpp"

namespace building_pcc_data
{
	int easy3d_fill_holes(easy3d::SurfaceMesh* mesh, const bool allow_internal_log_print)
	{
		LOG(INFO) << "	- simple fill holes ... ";
		easy3d::StopWatch w;
		w.start();
		// prepare some information about the smallest and largest holes to the user
		int min_hole_size = easy3d::max<int>();
		int max_hole_size = -easy3d::max<int>();
		std::vector< std::pair<easy3d::SurfaceMesh::Halfedge, int> > holes;

		auto visited = mesh->add_halfedge_property<bool>("DialogSurfaceMeshHoleFilling::h::visited", false);
		for (auto h : mesh->halfedges())
		{
			if (!visited[h] && mesh->is_border(h))
			{
				int size = 0;
				easy3d::SurfaceMesh::Halfedge hh = h;
				do {
					visited[hh] = true;
					++size;
					if (!mesh->is_manifold(mesh->target(hh)))
					{
						size += 123456;
						break;
					}
					hh = mesh->next(hh);
				} while (hh != h);

				min_hole_size = std::min(min_hole_size, size);
				max_hole_size = std::max(max_hole_size, size);
				if (size <= allowed_boundary_size)
				{
					holes.emplace_back(std::make_pair(h, size));
				}
			}
		}
		mesh->remove_halfedge_property(visited);

		// close holes whose sizes are smaller than the min allowed boundary size
		int num_closed = 0;

		for (const auto& hole : holes)
		{
			easy3d::SurfaceMeshHoleFilling hf(mesh);
			hf.fill_hole(hole.first);
			++num_closed;
		}

		if (allow_internal_log_print)
		{
			if (holes.empty())
			{
				if (min_hole_size == easy3d::max<int>() && max_hole_size == -easy3d::max<int>())
					LOG(WARNING) << "model is closed and no holes to fill";
				else
					LOG(WARNING) << "no holes meet the requirement (smallest: " << min_hole_size << ", largest: " << max_hole_size << ")";
			}
			else
			{
				LOG(INFO) << num_closed << " (out of " << holes.size() << ") holes filled";
			}
		}
		LOG(INFO) << "done in (s): " << w.time_string();
		return num_closed;
	}

	void holes_filling_preprocessing(easy3d::SurfaceMesh* mesh, std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map)
	{
		//if (!mesh->get_face_property<easy3d::vec3>("f:color"))
		//	mesh->add_face_property<easy3d::vec3>("f:color", easy3d::vec3(0.5882f, 0.5882f, 0.5882f));
		//auto fd_color = mesh->get_face_property<easy3d::vec3>("f:color");
		//const auto& pairs = easy3d::Surfacer::detect_self_intersections(mesh);
		//auto select = mesh->get_face_property<bool>("f:select");
		//if (select)
		//	select.vector().resize(mesh->n_faces(), false);
		//else
		//	select = mesh->add_face_property<bool>("f:select", false);
		//for (const auto& pair : pairs) {
		//	select[pair.first] = true;
		//	select[pair.second] = true;
		//}

		//for (auto& fd : mesh->faces())
		//{
		//	if (mesh->get_face_property<bool>("f:select")[fd])
		//	{
		//		fd_color[fd] = easy3d::vec3(1.0f, 1.0f, 0.0f);
		//	}
		//}

		//const std::string file_out1 = "C:/data/PhDthesis/git_pcc_data/data/exp/3d_geoinfo_paper_test/hole_steps/input_mesh_trans.ply";
		//easy3d::SurfaceMeshIO::save(file_out1, mesh);

		// pre-processing
		remesh_face_self_intersections(mesh);
		easy3d::Surfacer::stitch_borders(mesh);

		get_duplicated_vertices(mesh, duplicate_vd_ind_map);
		get_duplicated_edges(mesh, duplicate_vd_ind_map);
		get_face_with_properties(mesh);
	}

	void holes_filling_postprocessing(easy3d::SurfaceMesh* mesh)
	{
		stitch_and_orient_mesh(mesh);
		easy3d::Surfacer::merge_reversible_connected_components(mesh);
	}

	void get_two_edge_attributes
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Vertex& pre_vs,
		easy3d::SurfaceMesh::Vertex& pre_vt,
		easy3d::SurfaceMesh::Vertex& vs,
		easy3d::SurfaceMesh::Vertex& vt,
		easy3d::SurfaceMesh::Vertex& vd0,
		easy3d::SurfaceMesh::Vertex& vd1,
		easy3d::SurfaceMesh::Vertex& vd2,
		std::set<int>& two_edge_verts,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		int& pre_vs_ind,
		int& pre_vt_ind,
		int& vs_ind,
		int& vt_ind,
		bool& is_break_edge
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");

		int v0 = -1, v1 = -1, v2 = -1;
		pre_vs_ind = pre_vs.idx();
		pre_vt_ind = pre_vt.idx();
		vs_ind = vs.idx();
		vt_ind = vt.idx();

		two_edge_verts.insert(pre_vs.idx());
		two_edge_verts.insert(pre_vt.idx());
		two_edge_verts.insert(vs.idx());
		two_edge_verts.insert(vt.idx());

		if (is_duplicated_vertex[pre_vs])
		{
			pre_vs_ind = duplicate_vd_ind_map[pre_vs];
			two_edge_verts.insert(duplicated_vertex_ids[pre_vs].begin(), duplicated_vertex_ids[pre_vs].end());
		}

		if (is_duplicated_vertex[pre_vt])
		{
			pre_vt_ind = duplicate_vd_ind_map[pre_vt];
			two_edge_verts.insert(duplicated_vertex_ids[pre_vt].begin(), duplicated_vertex_ids[pre_vt].end());
		}

		if (is_duplicated_vertex[vs])
		{
			vs_ind = duplicate_vd_ind_map[vs];
			two_edge_verts.insert(duplicated_vertex_ids[vs].begin(), duplicated_vertex_ids[vs].end());
		}

		if (is_duplicated_vertex[vt])
		{
			vt_ind = duplicate_vd_ind_map[vt];
			two_edge_verts.insert(duplicated_vertex_ids[vt].begin(), duplicated_vertex_ids[vt].end());
		}

		vd0 = pre_vs;
		vd1 = pre_vt;
		vd2 = vs;
		v0 = pre_vs_ind;
		v1 = pre_vt_ind;
		if (vs_ind != v0 && vs_ind != v1 && (vt_ind == v0 || vt_ind == v1))
		{
			v2 = vs_ind;
		}
		else if (vt_ind != v0 && vt_ind != v1 && (vs_ind == v0 || vs_ind == v1))
		{
			v2 = vt_ind;
			vd2 = vt;
		}
		else
		{
			// break edges
			is_break_edge = true;
		}
	}

	double check_collinear_edges
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Vertex& vd0,
		easy3d::SurfaceMesh::Vertex& vd1,
		easy3d::SurfaceMesh::Vertex& vd2
	)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		easy3d::vec3 vd0vd1 = get_points_coord[vd1] - get_points_coord[vd0];
		easy3d::vec3 vd0vd2 = get_points_coord[vd2] - get_points_coord[vd0];
		double edge_angle = easy3d::geom::to_degrees(easy3d::geom::angle(vd0vd1, vd0vd2));
		if (edge_angle > 90 && edge_angle < 180)
			edge_angle = 180.0 - edge_angle;
		else if (edge_angle > 180.0)
			edge_angle -= 180.0;
		return edge_angle;
	}

	void get_two_edge_incident_facets
	(
		easy3d::SurfaceMesh* mesh,
		std::set<int>& two_edge_verts,
		std::set<int>& two_edge_incident_facets
	)
	{
		for (auto e_vi : two_edge_verts)
		{
			easy3d::SurfaceMesh::Vertex e_vd(e_vi);
			for (auto& e_fd : mesh->faces(e_vd))
				two_edge_incident_facets.insert(e_fd.idx());
		}
	}

	void create_face_from_two_edges
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Vertex& vd0,
		easy3d::SurfaceMesh::Vertex& vd1,
		easy3d::SurfaceMesh::Vertex& vd2,
		easy3d::vec3& p0,
		easy3d::vec3& p1,
		easy3d::vec3& p2
	)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		p0 = get_points_coord[vd0];
		p1 = get_points_coord[vd1];
		p2 = get_points_coord[vd2];
	}

	bool compute_face_intersections
	(
		easy3d::SurfaceMesh* mesh,
		std::set<int>& two_edge_incident_facets,
		easy3d::vec3& p0,
		easy3d::vec3& p1,
		easy3d::vec3& p2
	)
	{
		auto fd_plane = mesh->get_face_property<easy3d::Plane3>("f:face_plane");
		auto fd_cgal_tri2 = mesh->get_face_property<Triangle_2>("f:face_cgal_tri2");
		bool is_fd_intersected = false;
		for (auto& twoin_fd_i : two_edge_incident_facets)
		{
			easy3d::SurfaceMesh::Face in_fd(twoin_fd_i);//if
			double v0_dis = std::sqrt(fd_plane[in_fd].squared_distance(p0));
			double v1_dis = std::sqrt(fd_plane[in_fd].squared_distance(p1));
			double v2_dis = std::sqrt(fd_plane[in_fd].squared_distance(p2));
			double max_dis2plane = std::max(std::max(v0_dis, v1_dis), v2_dis);

			Polygon_2 intersected_poly_ef_if, intersected_poly_if_ef;
			Triangle_2 in_fd_tri = fd_cgal_tri2[in_fd];
			easy3d::Plane3  in_fd_plane = mesh->get_face_property<easy3d::Plane3>("f:face_plane")[in_fd];
			auto p0_proj = to_2d_new(in_fd_plane, plane_point(in_fd_plane), p0);
			auto p1_proj = to_2d_new(in_fd_plane, plane_point(in_fd_plane), p1);
			auto p2_proj = to_2d_new(in_fd_plane, plane_point(in_fd_plane), p2);
			Triangle_2 ef_if_proj_tri2(Point_2(p0_proj.x, p0_proj.y), Point_2(p1_proj.x, p1_proj.y), Point_2(p2_proj.x, p2_proj.y));
			triangle_2D_intersection_test(in_fd_tri, ef_if_proj_tri2, intersected_poly_ef_if);

			double intersect_poly_area_ef_if = std::abs(CGAL::to_double(intersected_poly_ef_if.area()));
			double ef_if_area_ratio = intersect_poly_area_ef_if / std::abs(CGAL::to_double(in_fd_tri.area()));

			if (max_dis2plane < point_to_plane_dis_thres && ef_if_area_ratio > intersected_area_ratio)
			{
				is_fd_intersected = true;
				break;
			}
		}

		return is_fd_intersected;
	}

	void get_hole_candidates
	(
		std::map<std::pair<int, int>, bool>& visited_ed,
		std::pair<int, int>& pre_ed_pair_1,
		std::pair<int, int>& pre_ed_pair_2,
		std::pair<int, int>& ed_pair_1,
		std::pair<int, int>& ed_pair_2,
		std::pair<int, int>& pre_h_pair,
		std::pair<int, int>& ed_h_pair,
		std::vector<std::pair<int, int>>& hole_edge_cur,
		std::map<int, int>& hole_v_count,
		int& pre_vs_ind,
		int& pre_vt_ind,
		int& vs_ind,
		int& vt_ind
	)
	{
		if (visited_ed.find(pre_ed_pair_1) == visited_ed.end() && visited_ed.find(pre_ed_pair_2) == visited_ed.end())
		{
			visited_ed[pre_ed_pair_1] = true;
			visited_ed[pre_ed_pair_2] = true;
			hole_edge_cur.push_back(pre_h_pair);
		}

		if (visited_ed.find(ed_pair_1) == visited_ed.end() && visited_ed.find(ed_pair_2) == visited_ed.end())
		{
			visited_ed[ed_pair_1] = true;
			visited_ed[ed_pair_2] = true;
			hole_edge_cur.push_back(ed_h_pair);
		}

		if (hole_v_count.find(pre_vs_ind) == hole_v_count.end())
			hole_v_count[pre_vs_ind] = 0;
		if (hole_v_count.find(pre_vt_ind) == hole_v_count.end())
			hole_v_count[pre_vt_ind] = 0;
		hole_v_count[pre_vs_ind] += 1;
		hole_v_count[pre_vt_ind] += 1;

		if (hole_v_count.find(vs_ind) == hole_v_count.end())
			hole_v_count[vs_ind] = 0;
		if (hole_v_count.find(vt_ind) == hole_v_count.end())
			hole_v_count[vt_ind] = 0;
		hole_v_count[vs_ind] += 1;
		hole_v_count[vt_ind] += 1;
	}

	void get_all_pre_h_candidates
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Halfedge& pre_h,
		easy3d::SurfaceMesh::Halfedge& hh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::map<std::pair<int, int>, int>& visited_edge_pair,
		std::set<int>& pre_h_candidates
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");

		// get turn vertex
		std::set<int> v_turn_ids;
		easy3d::SurfaceMesh::Edge pre_ed = mesh->edge(pre_h);
		easy3d::SurfaceMesh::Vertex pre_vs = mesh->vertex(pre_ed, 0);
		easy3d::SurfaceMesh::Vertex pre_vt = mesh->vertex(pre_ed, 1);

		easy3d::SurfaceMesh::Edge ed = mesh->edge(hh);
		easy3d::SurfaceMesh::Vertex vs = mesh->vertex(ed, 0);
		easy3d::SurfaceMesh::Vertex vt = mesh->vertex(ed, 1);

		int pre_vs_ind = pre_vs.idx();
		int pre_vt_ind = pre_vt.idx();
		int vs_ind = vs.idx();
		int vt_ind = vt.idx();

		if (is_duplicated_vertex[pre_vs])
			pre_vs_ind = duplicate_vd_ind_map[pre_vs];

		if (is_duplicated_vertex[pre_vt])
			pre_vt_ind = duplicate_vd_ind_map[pre_vt];

		if (is_duplicated_vertex[vs])
			vs_ind = duplicate_vd_ind_map[vs];

		if (is_duplicated_vertex[vt])
			vt_ind = duplicate_vd_ind_map[vt];

		easy3d::SurfaceMesh::Vertex v_turn(-1);
		if (pre_vs_ind == vs_ind || pre_vs_ind == vt_ind)
			v_turn = pre_vs;
		else if (pre_vt_ind == vs_ind || pre_vt_ind == vt_ind)
			v_turn = pre_vt;

		v_turn_ids.insert(v_turn.idx());
		if (is_duplicated_vertex[v_turn])
			v_turn_ids.insert(duplicated_vertex_ids[v_turn].begin(), duplicated_vertex_ids[v_turn].end());

		pre_h_candidates.insert(pre_h.idx());
		for (auto& v_ti : v_turn_ids)
		{
			easy3d::SurfaceMesh::Vertex v_turn(v_ti);
			for (auto& pre_hd_i : mesh->halfedges(v_turn))
			{
				std::pair<int, int> pre_hd_hd_pair1 = std::make_pair(pre_hd_i.idx(), hh.idx());
				std::pair<int, int> pre_hd_hd_pair2 = std::make_pair(hh.idx(), pre_hd_i.idx());

				if (pre_hd_i != pre_h && pre_hd_i != hh && mesh->is_border(pre_hd_i)
					&& ((visited_edge_pair.find(pre_hd_hd_pair1) == visited_edge_pair.end() && visited_edge_pair.find(pre_hd_hd_pair2) == visited_edge_pair.end()) 
						|| (visited_edge_pair[pre_hd_hd_pair1] < 2 && visited_edge_pair[pre_hd_hd_pair2] < 2))
					)
				{
					pre_h_candidates.insert(pre_hd_i.idx());
				}
			}
		}
	}

	void hole_detection_pipeline
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Halfedge& pre_h,
		easy3d::SurfaceMesh::Halfedge& hh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::map<std::pair<int, int>, bool>& visited_ed,
		std::map<int, int>& hole_v_count,
		std::vector<std::pair<int, int>>& hole_edge_cur
	)
	{
		auto is_degenerate_edge = mesh->get_edge_property<bool>("e:degenerate_edge");
		// collect all neighboring facets around the hole edges
		if (pre_h.idx() != -1 && !is_degenerate_edge[mesh->edge(pre_h)] && !is_degenerate_edge[mesh->edge(hh)])
		{
			easy3d::SurfaceMesh::Edge pre_ed = mesh->edge(pre_h);
			easy3d::SurfaceMesh::Vertex pre_vs = mesh->vertex(pre_ed, 0);
			easy3d::SurfaceMesh::Vertex pre_vt = mesh->vertex(pre_ed, 1);

			easy3d::SurfaceMesh::Edge ed = mesh->edge(hh);
			easy3d::SurfaceMesh::Vertex vs = mesh->vertex(ed, 0);
			easy3d::SurfaceMesh::Vertex vt = mesh->vertex(ed, 1);

			// get two edge attributes
			bool is_break_edge = false;
			easy3d::SurfaceMesh::Vertex vd0, vd1, vd2;
			std::pair<int, int> pre_ed_pair_1, pre_ed_pair_2, ed_pair_1, ed_pair_2;
			std::set<int> two_edge_verts, two_edge_incident_facets;
			int pre_vs_ind = -1, pre_vt_ind = -1, vs_ind = -1, vt_ind = -1;
			get_two_edge_attributes(mesh, pre_vs, pre_vt, vs, vt, vd0, vd1, vd2, two_edge_verts, duplicate_vd_ind_map, pre_vs_ind, pre_vt_ind, vs_ind, vt_ind, is_break_edge);

			if (!is_break_edge)
			{
				get_two_edge_incident_facets(mesh, two_edge_verts, two_edge_incident_facets);

				pre_ed_pair_1 = std::make_pair(pre_vs_ind, pre_vt_ind);
				pre_ed_pair_2 = std::make_pair(pre_vt_ind, pre_vs_ind);
				ed_pair_1 = std::make_pair(vs_ind, vt_ind);
				ed_pair_2 = std::make_pair(vt_ind, vs_ind);
				std::pair<int, int> pre_h_pair = std::make_pair(pre_vs.idx(), pre_vt.idx());
				std::pair<int, int> ed_h_pair = std::make_pair(vs.idx(), vt.idx());

				// create face from two edges
				easy3d::vec3 p0, p1, p2;
				create_face_from_two_edges(mesh, vd0, vd1, vd2, p0, p1, p2);

				// compute face intersections
				bool is_fd_intersected = compute_face_intersections(mesh, two_edge_incident_facets, p0, p1, p2);

				// no intersect are hole edges
				if (!is_fd_intersected)
				{
					get_hole_candidates(visited_ed, pre_ed_pair_1, pre_ed_pair_2, ed_pair_1, ed_pair_2, pre_h_pair, ed_h_pair, hole_edge_cur, hole_v_count, pre_vs_ind, pre_vt_ind, vs_ind, vt_ind);
				}
			}
		}
	}

	void add_missing_edge
	(
		std::map< std::pair<int, int>, bool>& visited_edges,
		std::vector<std::pair<int, int>>& hole_pts,
		const int v0_ind,
		const int v1_ind,
		const easy3d::SurfaceMesh::Vertex v0,
		const easy3d::SurfaceMesh::Vertex v1
	)
	{
		std::pair<int, int> v0v1 = std::make_pair(v0_ind, v1_ind);
		std::pair<int, int> v1v0 = std::make_pair(v1_ind, v0_ind);
		if (visited_edges.find(v0v1) == visited_edges.end() && visited_edges.find(v1v0) == visited_edges.end())
		{
			visited_edges[v0v1] = true;
			visited_edges[v1v0] = true;
			std::pair<int, int> new_h = std::make_pair(v0.idx(), v1.idx());
			hole_pts.push_back(new_h);
		}
	}

	void check_vertices_for_missing_edges
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::map< int, bool>& current_hole_vi_map,
		std::map< std::pair<int, int>, bool>& visited_edges,
		std::vector<std::pair<int, int>>& hole_pts,
		const easy3d::vec3 vsvt_vec,
		const int v_i
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		easy3d::SurfaceMesh::Vertex v_c(v_i);
		for (auto& hd : mesh->halfedges(v_c))
		{
			easy3d::SurfaceMesh::Edge ed = mesh->edge(hd);
			if (mesh->is_border(ed))
			{
				easy3d::SurfaceMesh::Vertex v0 = mesh->vertex(ed, 0);
				easy3d::SurfaceMesh::Vertex v1 = mesh->vertex(ed, 1);

				int v0_ind = v0.idx();
				int v1_ind = v1.idx();
				if (is_duplicated_vertex[v0])
					v0_ind = duplicate_vd_ind_map[v0];
				if (is_duplicated_vertex[v1])
					v1_ind = duplicate_vd_ind_map[v1];

				if (current_hole_vi_map.find(v0_ind) != current_hole_vi_map.end()
					&& current_hole_vi_map.find(v1_ind) != current_hole_vi_map.end())
				{
					add_missing_edge(visited_edges, hole_pts, v0_ind, v1_ind, v0, v1);
				}
				else if ((current_hole_vi_map.find(v0_ind) != current_hole_vi_map.end() && current_hole_vi_map.find(v1_ind) == current_hole_vi_map.end())
					|| (current_hole_vi_map.find(v0_ind) == current_hole_vi_map.end() && current_hole_vi_map.find(v1_ind) != current_hole_vi_map.end()))
				{
					easy3d::vec3 v0v1_vec = get_points_coord[v0] - get_points_coord[v1];
					double edge_angle = easy3d::geom::to_degrees(easy3d::geom::angle(vsvt_vec, v0v1_vec));
					if (edge_angle > 90 && edge_angle < 180)
						edge_angle = 180.0 - edge_angle;
					else if (edge_angle > 180.0)
						edge_angle -= 180.0;

					if (edge_angle < edge_overlap_angle)
					{
						add_missing_edge(visited_edges, hole_pts, v0_ind, v1_ind, v0, v1);
					}
				}
			}
		}
	}

	void find_missing_edges_in_holes
	(
		easy3d::SurfaceMesh* mesh,
		std::vector<std::pair<int, int>>& holes_edges_cur,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");

		std::map< std::pair<int, int>, bool> visited_edges;
		std::map< int, bool> current_hole_vi_map;
		for (int i = 0; i < holes_edges_cur.size(); ++i)
		{
			easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[i].first);
			easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[i].second);
			int vs_ind = vs.idx();
			int vt_ind = vt.idx();
			if (is_duplicated_vertex[vs])
				vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				vt_ind = duplicate_vd_ind_map[vt];

			std::pair<int, int> h_pair_1 = std::make_pair(vs_ind, vt_ind);
			std::pair<int, int> h_pair_2 = std::make_pair(vt_ind, vs_ind);
			visited_edges[h_pair_1] = true;
			visited_edges[h_pair_2] = true;

			current_hole_vi_map[vs_ind] = true;
			current_hole_vi_map[vt_ind] = true;
		}

		for (int i = 0; i < holes_edges_cur.size(); ++i)
		{
			std::set<int> vs_set, vt_set;
			easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[i].first);
			easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[i].second);
			vs_set.insert(vs.idx());
			vt_set.insert(vt.idx());
			easy3d::vec3 vsvt_vec = get_points_coord[vt] - get_points_coord[vs];

			if (is_duplicated_vertex[vs])
				vs_set.insert(duplicated_vertex_ids[vs].begin(), duplicated_vertex_ids[vs].end());
			if (is_duplicated_vertex[vt])
				vt_set.insert(duplicated_vertex_ids[vt].begin(), duplicated_vertex_ids[vt].end());

			for (auto& vs_i : vs_set)
			{
				check_vertices_for_missing_edges(mesh, duplicate_vd_ind_map, current_hole_vi_map, visited_edges, holes_edges_cur, vsvt_vec, vs_i);
			}

			for (auto& vt_i : vt_set)
			{
				check_vertices_for_missing_edges(mesh, duplicate_vd_ind_map, current_hole_vi_map, visited_edges, holes_edges_cur, vsvt_vec, vt_i);
			}
		}
	}

	bool check_visited_turn
	(
		std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, bool>& visited_turn,
		std::pair<int, int>& h_pair_1,
		std::pair<int, int>& h_pair_2,
		std::pair<int, int>& pre_h_pair_1,
		std::pair<int, int>& pre_h_pair_2
	)
	{
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_1 = std::make_pair(h_pair_1, pre_h_pair_1);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_2 = std::make_pair(h_pair_2, pre_h_pair_2);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_3 = std::make_pair(h_pair_1, pre_h_pair_2);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_4 = std::make_pair(h_pair_2, pre_h_pair_1);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_5 = std::make_pair(pre_h_pair_1, h_pair_1);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_6 = std::make_pair(pre_h_pair_2, h_pair_2);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_7 = std::make_pair(pre_h_pair_2, h_pair_1);
		std::pair<std::pair<int, int>, std::pair<int, int>> turn_pair_8 = std::make_pair(pre_h_pair_1, h_pair_2);
		if (visited_turn.find(turn_pair_1) == visited_turn.end()
			&& visited_turn.find(turn_pair_2) == visited_turn.end()
			&& visited_turn.find(turn_pair_3) == visited_turn.end()
			&& visited_turn.find(turn_pair_4) == visited_turn.end()
			&& visited_turn.find(turn_pair_5) == visited_turn.end()
			&& visited_turn.find(turn_pair_6) == visited_turn.end()
			&& visited_turn.find(turn_pair_7) == visited_turn.end()
			&& visited_turn.find(turn_pair_8) == visited_turn.end()
			)
		{
			visited_turn[turn_pair_1] = true;
			visited_turn[turn_pair_2] = true;
			visited_turn[turn_pair_3] = true;
			visited_turn[turn_pair_4] = true;
			visited_turn[turn_pair_5] = true;
			visited_turn[turn_pair_6] = true;
			visited_turn[turn_pair_7] = true;
			visited_turn[turn_pair_8] = true;

			return false;
		}
		else
		{
			return true;
		}
	}

	void re_order_hole_vertices
	(
		easy3d::SurfaceMesh* mesh,
		std::vector<std::pair<int, int>>& holes_edges_cur,
		std::vector<std::vector<std::pair<int, int>>>& holes_edges_new_vec,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::map<std::pair<int, int>, int>& visited_edge_pair,
		std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, bool>& visited_turn
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");

		
		for (int i = 0; i < holes_edges_cur.size(); ++i)
		{
			bool complete_loop = false, visited_turn_break = false;
			int pre_vs_ind = -1, pre_vt_ind = -1;
			std::vector<std::pair<int, int>> holes_edges_new;
			std::map< std::pair<int, int>, bool> local_visited_edges;
			std::map<int, int> local_visited_vert;

			easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[i].first);
			easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[i].second);
			int first_vs_ind = vs.idx();
			int first_vt_ind = vt.idx();
			if (is_duplicated_vertex[vs])
				first_vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				first_vt_ind = duplicate_vd_ind_map[vt];

			local_visited_vert[first_vs_ind] = 1;
			local_visited_vert[first_vt_ind] = 1;
			std::pair<int, int> first_h_pair_1 = std::make_pair(first_vs_ind, first_vt_ind);
			std::pair<int, int> first_h_pair_2 = std::make_pair(first_vt_ind, first_vs_ind);
			local_visited_edges[first_h_pair_1] = true;
			local_visited_edges[first_h_pair_2] = true;

			if ((visited_edge_pair.find(first_h_pair_1) == visited_edge_pair.end() && visited_edge_pair.find(first_h_pair_2) == visited_edge_pair.end())
				|| (visited_edge_pair[first_h_pair_1] < 2 && visited_edge_pair[first_h_pair_2] < 2))
			{
				holes_edges_new.push_back(holes_edges_cur[i]);
				std::stack<int> cur_edge;
				cur_edge.push(i);
				pre_vs_ind = first_vs_ind;
				pre_vt_ind = first_vt_ind;
				std::pair<int, int> pre_h_pair_1 = first_h_pair_1;
				std::pair<int, int> pre_h_pair_2 = first_h_pair_2;

				while (!cur_edge.empty())
				{
					int top_i = cur_edge.top();
					cur_edge.pop();
					bool found_connceted_edge = false;

					for (int j = 0; j < holes_edges_cur.size(); ++j)
					{
						if (j == top_i)
							continue;
						easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[j].first);
						easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[j].second);
						int vs_ind = vs.idx();
						int vt_ind = vt.idx();
						if (is_duplicated_vertex[vs])
							vs_ind = duplicate_vd_ind_map[vs];
						if (is_duplicated_vertex[vt])
							vt_ind = duplicate_vd_ind_map[vt];

						std::pair<int, int> h_pair_1 = std::make_pair(vs_ind, vt_ind);
						std::pair<int, int> h_pair_2 = std::make_pair(vt_ind, vs_ind);
						if (local_visited_edges.find(h_pair_1) != local_visited_edges.end() || local_visited_edges.find(h_pair_2) != local_visited_edges.end())
							continue;

						if (local_visited_vert.find(vs_ind) != local_visited_vert.end() && local_visited_vert[vs_ind] == 2)
							continue;

						if (local_visited_vert.find(vt_ind) != local_visited_vert.end() && local_visited_vert[vt_ind] == 2)
							continue;

						found_connceted_edge = false;
						if (vs_ind != pre_vs_ind && vs_ind != pre_vt_ind && (vt_ind == pre_vs_ind || vt_ind == pre_vt_ind))
						{
							if (vs_ind == first_vs_ind || vs_ind == first_vt_ind)
								complete_loop = true;

							found_connceted_edge = true;
						}
						else if (vt_ind != pre_vs_ind && vt_ind != pre_vt_ind && (vs_ind == pre_vs_ind || vs_ind == pre_vt_ind))
						{
							if (vt_ind == first_vs_ind || vt_ind == first_vt_ind)
								complete_loop = true;

							found_connceted_edge = true;
						}

						if (found_connceted_edge)
						{
							visited_turn_break = check_visited_turn(visited_turn, h_pair_1, h_pair_2, pre_h_pair_1, pre_h_pair_2);
							
							if (visited_edge_pair.find(h_pair_1) == visited_edge_pair.end()
								&& visited_edge_pair.find(h_pair_2) == visited_edge_pair.end())
							{
								visited_edge_pair[h_pair_1] = 1;
								visited_edge_pair[h_pair_2] = 1;
							}
							else if (visited_edge_pair[h_pair_1] < 2 && visited_edge_pair[h_pair_2] < 2)
							{
								visited_edge_pair[h_pair_1] += 1;
								visited_edge_pair[h_pair_2] += 1;
							}

							local_visited_edges[h_pair_1] = true;
							local_visited_edges[h_pair_2] = true;
							if (local_visited_vert.find(vs_ind) == local_visited_vert.end())
								local_visited_vert[vs_ind] = 1;
							else
								local_visited_vert[vs_ind] += 1;

							if (local_visited_vert.find(vt_ind) == local_visited_vert.end())
								local_visited_vert[vt_ind] = 1;
							else
								local_visited_vert[vt_ind] += 1;

							holes_edges_new.push_back(holes_edges_cur[j]);
							pre_vs_ind = vs_ind;
							pre_vt_ind = vt_ind;
							cur_edge.push(j);

							pre_h_pair_1 = h_pair_1;
							pre_h_pair_2 = h_pair_2;
							break;
						}
					}

					if (complete_loop || visited_turn_break || !found_connceted_edge)
						break;
				}
			}

			if (holes_edges_new.size() > 2 && !visited_turn_break)
				holes_edges_new_vec.push_back(holes_edges_new);
		}
	}


	bool planar_hole_checking
	(
		easy3d::SurfaceMesh* mesh,
		std::vector<std::pair<int, int>>& holes_edges_cur,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		easy3d::Plane3& proj_plane,
		std::map<int, bool>& visited_vd
	)
	{
		const easy3d::vec3& p_orig = mesh->points()[0];
		auto get_mesh_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		
		int v_num = 0;
		easy3d::PrincipalAxes<3> pca;
		pca.begin();
		for (int hpi = 0; hpi < holes_edges_cur.size(); hpi++)
		{
			easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[hpi].first);
			easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[hpi].second);
			int vs_ind = vs.idx();
			int vt_ind = vt.idx();
			if (is_duplicated_vertex[vs])
				vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				vt_ind = duplicate_vd_ind_map[vt];

			if (visited_vd.find(vs_ind) == visited_vd.end())
			{
				visited_vd[vs_ind] = true;
				pca.add(get_mesh_points_coord[vs] - p_orig);
				++v_num;
			}

			if (visited_vd.find(vt_ind) == visited_vd.end())
			{
				visited_vd[vt_ind] = true;
				pca.add(get_mesh_points_coord[vt] - p_orig);
				++v_num;
			}
		}
		pca.end();
		proj_plane = easy3d::Plane3(pca.center<float>(), pca.axis<float>(2));
		double sphericity = float(pca.eigen_value(2) / pca.eigen_value(1));

		if (sphericity > planar_hole_thres)
			return false;
		else
			return true;
	}

	easy3d::SurfaceMesh::Vertex get_non_adjacent_vertex
	(
		easy3d::SurfaceMesh* mesh,
		easy3d::SurfaceMesh::Vertex pre_vs,
		easy3d::SurfaceMesh::Vertex pre_vt,
		easy3d::SurfaceMesh::Vertex vs,
		easy3d::SurfaceMesh::Vertex vt,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");
		int pre_vs_ind = pre_vs.idx();
		int pre_vt_ind = pre_vt.idx();
		if (is_duplicated_vertex[pre_vs])
			pre_vs_ind = duplicate_vd_ind_map[pre_vs];
		if (is_duplicated_vertex[pre_vt])
			pre_vt_ind = duplicate_vd_ind_map[pre_vt];

		int vs_ind = vs.idx();
		int vt_ind = vt.idx();
		if (is_duplicated_vertex[vs])
			vs_ind = duplicate_vd_ind_map[vs];
		if (is_duplicated_vertex[vt])
			vt_ind = duplicate_vd_ind_map[vt];

		easy3d::SurfaceMesh::Vertex non_adjacent_v(vt.idx());
		if ((vs_ind == pre_vs_ind || vs_ind == pre_vt_ind) && vt_ind != pre_vs_ind && vt_ind != pre_vt_ind)
		{
			non_adjacent_v = vt;
		}
		else if ((vt_ind == pre_vs_ind || vt_ind == pre_vt_ind) && vs_ind != pre_vs_ind && vs_ind != pre_vt_ind)
		{
			non_adjacent_v = vs;
		}

		return non_adjacent_v;

	}

	void get_hole_start_end_vertex
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		easy3d::SurfaceMesh::Vertex& cur_start_v,
		easy3d::SurfaceMesh::Vertex& cur_end_v,
		easy3d::SurfaceMesh::Vertex& pre_vs,
		easy3d::SurfaceMesh::Vertex& pre_vt,
		std::map<int, bool>& visited_vd,
		const std::vector<std::pair<int, int>> holes_edges_cur
	)
	{
		int h_size = holes_edges_cur.size();
		easy3d::SurfaceMesh::Vertex vs0(holes_edges_cur[0].first);
		easy3d::SurfaceMesh::Vertex vt0(holes_edges_cur[0].second);
		easy3d::SurfaceMesh::Vertex vs1(holes_edges_cur[1].first);
		easy3d::SurfaceMesh::Vertex vt1(holes_edges_cur[1].second);
		cur_start_v = get_non_adjacent_vertex(mesh, vs1, vt1, vs0, vt0, duplicate_vd_ind_map);
		visited_vd[cur_start_v.idx()] = false;

		easy3d::SurfaceMesh::Vertex pre_pre_vs(holes_edges_cur[h_size - 2].first);
		easy3d::SurfaceMesh::Vertex pre_pre_vt(holes_edges_cur[h_size - 2].second);
		pre_vs = easy3d::SurfaceMesh::Vertex(holes_edges_cur[h_size - 1].first);
		pre_vt = easy3d::SurfaceMesh::Vertex(holes_edges_cur[h_size - 1].second);
		cur_end_v = get_non_adjacent_vertex(mesh, pre_pre_vs, pre_pre_vt, pre_vs, pre_vt, duplicate_vd_ind_map);
	}

	void reverse_hole_edge_vertices(std::vector<std::pair<int, int>>& holes_edges_cur)
	{
		std::vector<std::pair<int, int>> holes_edges_rev;
		for (int i = holes_edges_cur.size() - 1; i >= 0; --i)
			holes_edges_rev.push_back(holes_edges_cur[i]);

		holes_edges_cur.clear();
		holes_edges_cur.insert(holes_edges_cur.end(), holes_edges_rev.begin(), holes_edges_rev.end());
	}

	void planar_hole_edge_complement
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::map<int, bool>& visited_vd,
		std::vector<std::pair<int, int>>& holes_edges_cur,
		easy3d::SurfaceMesh::Vertex pre_vs,
		easy3d::SurfaceMesh::Vertex pre_vt,
		const easy3d::SurfaceMesh::Vertex cur_start_v,
		const easy3d::SurfaceMesh::Vertex cur_end_v,
		const easy3d::Plane3 proj_plane,
		const bool is_planar
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");

		std::stack<int> seed_v_stack;
		seed_v_stack.push(cur_end_v.idx());
		bool is_complete_loop = false;
		while (!seed_v_stack.empty())
		{
			easy3d::SurfaceMesh::Vertex top_v(seed_v_stack.top());
			std::set<int> seed_v_set = { top_v.idx() };
			if (is_duplicated_vertex[top_v])
				seed_v_set.insert(duplicated_vertex_ids[top_v].begin(), duplicated_vertex_ids[top_v].end());
			seed_v_stack.pop();

			std::pair<int, double> sel_vd_mindis(-1, FLT_MAX);
			for (auto& v_i : seed_v_set)
			{
				easy3d::SurfaceMesh::Vertex vd(v_i);
				for (auto& hd : mesh->halfedges(vd))
				{
					easy3d::SurfaceMesh::Edge ed = mesh->edge(hd);
					easy3d::SurfaceMesh::Vertex vs = mesh->vertex(ed, 0);
					easy3d::SurfaceMesh::Vertex vt = mesh->vertex(ed, 1);
					easy3d::SurfaceMesh::Face fd0 = mesh->face(ed, 0);
					easy3d::SurfaceMesh::Face fd1 = mesh->face(ed, 1);

					easy3d::SurfaceMesh::Vertex used_vd = vs;
					if (vs.idx() == vd.idx())
						used_vd = vt;

					int used_vd_ind = used_vd.idx();
					if (is_duplicated_vertex[used_vd])
						used_vd_ind = duplicate_vd_ind_map[used_vd];

					if (visited_vd.find(used_vd_ind) == visited_vd.end() || !visited_vd[used_vd_ind])
					{
						double pt2plane_dis = std::sqrt(proj_plane.squared_distance(get_points_coord[used_vd]));
						double pt2start_dis = easy3d::distance(get_points_coord[cur_start_v], get_points_coord[used_vd]);

						if (is_planar && pt2plane_dis < planar_hole_point_to_plane_dis_thres)
						{
							// get two edge attributes
							bool is_break_edge = false;
							easy3d::SurfaceMesh::Vertex vd0, vd1, vd2;
							std::pair<int, int> pre_ed_pair_1, pre_ed_pair_2, ed_pair_1, ed_pair_2;
							std::set<int> two_edge_verts, two_edge_incident_facets;
							int pre_vs_ind = -1, pre_vt_ind = -1, vs_ind = -1, vt_ind = -1;
							get_two_edge_attributes(mesh, pre_vs, pre_vt, vs, vt, vd0, vd1, vd2, two_edge_verts, duplicate_vd_ind_map, pre_vs_ind, pre_vt_ind, vs_ind, vt_ind, is_break_edge);

							if (!is_break_edge)
							{
								get_two_edge_incident_facets(mesh, two_edge_verts, two_edge_incident_facets);

								pre_ed_pair_1 = std::make_pair(pre_vs_ind, pre_vt_ind);
								pre_ed_pair_2 = std::make_pair(pre_vt_ind, pre_vs_ind);
								ed_pair_1 = std::make_pair(vs_ind, vt_ind);
								ed_pair_2 = std::make_pair(vt_ind, vs_ind);
								std::pair<int, int> pre_h_pair = std::make_pair(pre_vs.idx(), pre_vt.idx());
								std::pair<int, int> ed_h_pair = std::make_pair(vs.idx(), vt.idx());

								// create face from two edges
								easy3d::vec3 p0, p1, p2;
								create_face_from_two_edges(mesh, vd0, vd1, vd2, p0, p1, p2);

								// compute face intersections
								bool is_fd_intersected = compute_face_intersections(mesh, two_edge_incident_facets, p0, p1, p2);

								// no intersect are hole edges
								if (!is_fd_intersected)
								{
									if (pt2start_dis < sel_vd_mindis.second)
									{
										sel_vd_mindis.first = used_vd.idx();
										sel_vd_mindis.second = pt2start_dis;
									}
								}
							}
						}
					}
				}
			}

			if ((visited_vd.find(sel_vd_mindis.first) == visited_vd.end() || !visited_vd[sel_vd_mindis.first]) && sel_vd_mindis.first != -1)
			{
				holes_edges_cur.push_back(std::make_pair(top_v.idx(), sel_vd_mindis.first));
				pre_vs = top_v;
				pre_vt = easy3d::SurfaceMesh::Vertex(sel_vd_mindis.first);
				visited_vd[sel_vd_mindis.first] = true;
				seed_v_stack.push(sel_vd_mindis.first);

				is_complete_loop = check_hole_loop_completeness(holes_edges_cur);
				if (is_complete_loop)
					break;
			}
			else
			{
				break;
			}
		}
	}

	bool hole_edge_collinear_check
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		const std::vector<std::pair<int, int>> holes_edges_cur
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		bool is_collinear = false;
		int collinear_count = 0;
		for (int i = 1; i < holes_edges_cur.size(); ++i)
		{
			easy3d::SurfaceMesh::Vertex pre_vs(holes_edges_cur[i - 1].first);
			easy3d::SurfaceMesh::Vertex pre_vt(holes_edges_cur[i - 1].second);
			easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[i].first);
			easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[i].second);
			
			int pre_vs_ind = pre_vs.idx();
			int pre_vt_ind = pre_vt.idx();
			int vs_ind = vs.idx();
			int vt_ind = vt.idx();
			if (is_duplicated_vertex[pre_vs])
				pre_vs_ind = duplicate_vd_ind_map[pre_vs];
			if (is_duplicated_vertex[pre_vt])
				pre_vt_ind = duplicate_vd_ind_map[pre_vt];
			if (is_duplicated_vertex[vs])
				vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				vt_ind = duplicate_vd_ind_map[vt];

			easy3d::SurfaceMesh::Vertex vd0 = pre_vs;
			easy3d::SurfaceMesh::Vertex vd1 = pre_vt;
			easy3d::SurfaceMesh::Vertex vd2 = vs;
			if (vt_ind != pre_vs_ind && vt_ind != pre_vt_ind && (vs_ind == pre_vs_ind || vs_ind == pre_vt_ind))
				vd2 = vt;

			double edge_angle = std::abs(check_collinear_edges(mesh, vd0, vd1, vd2));

			if (edge_angle < edge_overlap_angle)
			{
				++collinear_count;
			}
		}

		if (collinear_count == holes_edges_cur.size() - 1)
			is_collinear = true;

		return is_collinear;
	}

	void hole_edge_complement
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::vector<std::vector<std::pair<int, int>>>& incomplete_holes_edges,
		std::vector<std::vector<std::pair<int, int>>>& holes_edges
	)
	{
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		
		for (int i = 0; i < incomplete_holes_edges.size(); ++i)
		{
			std::vector<std::pair<int, int>> holes_edges_cur = incomplete_holes_edges[i];
			easy3d::Plane3 proj_plane;
			std::map<int, bool> visited_vd;
			bool is_planar = planar_hole_checking(mesh, holes_edges_cur, duplicate_vd_ind_map, proj_plane, visited_vd);

			easy3d::SurfaceMesh::Vertex cur_start_v, cur_end_v, pre_vs, pre_vt;
			get_hole_start_end_vertex(mesh, duplicate_vd_ind_map, cur_start_v, cur_end_v, pre_vs, pre_vt, visited_vd, holes_edges_cur);

			if (!is_planar)
			{
				holes_edges_cur.push_back(std::make_pair(cur_start_v.idx(), cur_end_v.idx()));
				holes_edges.push_back(holes_edges_cur);
				continue;
			}

			planar_hole_edge_complement(mesh, duplicate_vd_ind_map, visited_vd, holes_edges_cur, pre_vs, pre_vt, cur_start_v, cur_end_v, proj_plane, is_planar);

			reverse_hole_edge_vertices(holes_edges_cur);
			get_hole_start_end_vertex(mesh, duplicate_vd_ind_map, cur_start_v, cur_end_v, pre_vs, pre_vt, visited_vd, holes_edges_cur);
			planar_hole_edge_complement(mesh, duplicate_vd_ind_map, visited_vd, holes_edges_cur, pre_vs, pre_vt, cur_start_v, cur_end_v, proj_plane, is_planar);

			//if (is_complete_loop)
			if (holes_edges_cur.size() > 2)
			{
				bool is_collinear_hole = hole_edge_collinear_check(mesh, duplicate_vd_ind_map, holes_edges_cur);
				if (!is_collinear_hole)
					holes_edges.push_back(holes_edges_cur);
			}
		}
	}

	bool check_hole_loop_completeness(std::vector<std::pair<int, int>>& hole_loop)
	{
		int end_i = hole_loop.size() - 1;
		if (hole_loop[end_i].first == hole_loop[0].first
			|| hole_loop[end_i].first == hole_loop[0].second
			|| hole_loop[end_i].second == hole_loop[0].first
			|| hole_loop[end_i].second == hole_loop[0].second)
			return true;
		else
			return false;
	}

	void filter_invalid_hole_edges
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::vector<std::vector<std::pair<int, int>>>& holes_edges_new_vec,
		std::vector<std::vector<std::pair<int, int>>>& holes_edges
	)
	{
		std::vector<std::vector<std::pair<int, int>>> incomplete_holes_edges;
		for (int i = 0; i < holes_edges_new_vec.size(); ++i)
		{
			std::vector<std::pair<int, int>> holes_edges_cur_new;
			std::vector<std::pair<int, int>> holes_edges_cur = holes_edges_new_vec[i];
			for (int j = 1; j < holes_edges_cur.size(); ++j)
			{
				easy3d::SurfaceMesh::Vertex pre_vs(holes_edges_cur[j - 1].first);
				easy3d::SurfaceMesh::Vertex pre_vt(holes_edges_cur[j - 1].second);
				easy3d::SurfaceMesh::Vertex vs(holes_edges_cur[j].first);
				easy3d::SurfaceMesh::Vertex vt(holes_edges_cur[j].second);

				// get two edge attributes
				bool is_break_edge = false;
				easy3d::SurfaceMesh::Vertex vd0, vd1, vd2;
				std::pair<int, int> pre_ed_pair_1, pre_ed_pair_2, ed_pair_1, ed_pair_2;
				std::set<int> two_edge_verts, two_edge_incident_facets;
				int pre_vs_ind = -1, pre_vt_ind = -1, vs_ind = -1, vt_ind = -1;
				get_two_edge_attributes(mesh, pre_vs, pre_vt, vs, vt, vd0, vd1, vd2, two_edge_verts, duplicate_vd_ind_map, pre_vs_ind, pre_vt_ind, vs_ind, vt_ind, is_break_edge);

				if (!is_break_edge)
				{
					get_two_edge_incident_facets(mesh, two_edge_verts, two_edge_incident_facets);

					pre_ed_pair_1 = std::make_pair(pre_vs_ind, pre_vt_ind);
					pre_ed_pair_2 = std::make_pair(pre_vt_ind, pre_vs_ind);
					ed_pair_1 = std::make_pair(vs_ind, vt_ind);
					ed_pair_2 = std::make_pair(vt_ind, vs_ind);
					std::pair<int, int> pre_h_pair = std::make_pair(pre_vs.idx(), pre_vt.idx());
					std::pair<int, int> ed_h_pair = std::make_pair(vs.idx(), vt.idx());

					// create face from two edges
					easy3d::vec3 p0, p1, p2;
					create_face_from_two_edges(mesh, vd0, vd1, vd2, p0, p1, p2);

					// compute face intersections
					bool is_fd_intersected = compute_face_intersections(mesh, two_edge_incident_facets, p0, p1, p2);

					// no intersect are hole edges
					if (!is_fd_intersected)
					{
						holes_edges_cur_new.push_back(holes_edges_cur[j - 1]);
						if (j == holes_edges_cur.size() - 1)
						{
							holes_edges_cur_new.push_back(holes_edges_cur[j]);
							bool is_complete_loop = check_hole_loop_completeness(holes_edges_cur_new);
							
							if (holes_edges_cur_new.size() > 2)
							{
								if (is_complete_loop)
									holes_edges.push_back(holes_edges_cur_new);
								else
									incomplete_holes_edges.push_back(holes_edges_cur_new);
							}
								
							holes_edges_cur_new.clear();
						}
					}
					else if (is_fd_intersected)
					{
						holes_edges_cur_new.push_back(holes_edges_cur[j - 1]);
						if (holes_edges_cur_new.size() > 2)
							incomplete_holes_edges.push_back(holes_edges_cur_new);//holes_edges.push_back(holes_edges_cur_new);//
						holes_edges_cur_new.clear();
					}
				}
			}
		}

		if (!incomplete_holes_edges.empty())
			hole_edge_complement(mesh, duplicate_vd_ind_map, incomplete_holes_edges, holes_edges);
	}

	void complex_holes_detection
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::vector<std::vector<std::pair<int, int>>>& holes_edges,
		int& min_hole_size,
		int& max_hole_size
	)
	{
		auto is_degenerate_edge = mesh->get_edge_property<bool>("e:degenerate_edge");
		auto visited = mesh->add_halfedge_property<bool>("DialogSurfaceMeshHoleFilling::h::visited", false);
		min_hole_size = easy3d::max<int>();
		max_hole_size = -easy3d::max<int>();
		std::map<std::pair<int, int>, int> visited_edge_pair;
		std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, bool> visited_turn;
		for (auto h : mesh->halfedges())
		{
			if (!visited[h] && mesh->is_border(h))
			{
				int size = 0;
				std::map<int, int> hole_v_count;
				std::map<std::pair<int, int>, bool> visited_ed;
				std::vector<std::pair<int, int>> holes_edges_cur;
				easy3d::SurfaceMesh::Halfedge pre_h(-1);
				easy3d::SurfaceMesh::Halfedge hh = h;
				do {
					visited[hh] = true;
					++size;
					if (!mesh->is_manifold(mesh->target(hh)))
					{
						size += 123456;
						break;
					}

					if (pre_h.idx() != -1 && !is_degenerate_edge[mesh->edge(pre_h)] && !is_degenerate_edge[mesh->edge(hh)])
					{
						// get all adjacent halfedges
						std::set<int> pre_h_candidates;
						get_all_pre_h_candidates(mesh, pre_h, hh, duplicate_vd_ind_map, visited_edge_pair, pre_h_candidates);

						for (auto pre_hd_i : pre_h_candidates)
						{
							easy3d::SurfaceMesh::Halfedge pre_hd_c(pre_hd_i);
							// hole detection pipeline
							hole_detection_pipeline(mesh, pre_hd_c, hh, duplicate_vd_ind_map, visited_ed, hole_v_count, holes_edges_cur);
						}
					}

					pre_h = hh;
					hh = mesh->next(hh);
				} while (hh != h);

				min_hole_size = std::min(min_hole_size, size);
				max_hole_size = std::max(max_hole_size, size);

				if (!holes_edges_cur.empty())
				{
					find_missing_edges_in_holes(mesh, holes_edges_cur, duplicate_vd_ind_map);
					std::vector<std::vector<std::pair<int, int>>> holes_edges_new_vec;
					re_order_hole_vertices(mesh, holes_edges_cur, holes_edges_new_vec, duplicate_vd_ind_map, visited_edge_pair, visited_turn);
					filter_invalid_hole_edges(mesh, duplicate_vd_ind_map, holes_edges_new_vec, holes_edges);
				}
			}
		}
		mesh->remove_halfedge_property(visited);
	}

	void copy_mesh(easy3d::SurfaceMesh* mesh, easy3d::SurfaceMeshBuilder& builder)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		for (auto& vd : mesh->vertices())
			builder.add_vertex(get_points_coord[vd]);
		for (auto& fd : mesh->faces())
		{
			std::vector<easy3d::SurfaceMesh::Vertex> fd_verts;
			for (auto& vd : mesh->vertices(fd))
			{
				fd_verts.push_back(vd);
			}
			builder.add_face(fd_verts);
		}
	}

	void project_hole_points
	(
		easy3d::SurfaceMesh* mesh,
		std::vector<std::pair<int, int>>& hole_edges,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::vector<CDPoint_in> &hole_points,
		std::map<int, std::pair<easy3d::vec3, easy3d::vec2>>& vd_2d3d_map
	)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		easy3d::PrincipalAxes<3> pca;
		pca.begin();
		for (int hpi = 0; hpi < hole_edges.size(); hpi++)
		{
			easy3d::SurfaceMesh::Vertex vs(hole_edges[hpi].first);
			easy3d::SurfaceMesh::Vertex vt(hole_edges[hpi].second);
			int vs_ind = vs.idx();
			int vt_ind = vt.idx();
			if (is_duplicated_vertex[vs])
				vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				vt_ind = duplicate_vd_ind_map[vt];

			if (vd_2d3d_map.find(vs_ind) == vd_2d3d_map.end())
			{
				auto p_vs = get_points_coord[vs];
				vd_2d3d_map[vs_ind] = std::make_pair(p_vs, easy3d::vec2());
				pca.add(p_vs);
			}

			if (vd_2d3d_map.find(vt_ind) == vd_2d3d_map.end())
			{
				auto p_vt = get_points_coord[vt];
				vd_2d3d_map[vt_ind] = std::make_pair(p_vt, easy3d::vec2());
				pca.add(p_vt);
			}
		}
		pca.end();
		easy3d::Plane3 proj_plane = easy3d::Plane3(pca.center<float>(), pca.axis<float>(2));

		for (auto& vd_map : vd_2d3d_map)
			vd_map.second.second = to_2d_new(proj_plane, plane_point(proj_plane), vd_map.second.first);
	}

	bool extract_polyline_from_holes
	(
		easy3d::SurfaceMesh* mesh,
		std::vector<std::pair<int, int>>& hole_edges,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		Polygon_2_in& polygon2d_constraints,
		std::vector< std::pair<CDPoint_in, unsigned> >& poly_points_pairs,
		std::map<int, int>& poly_index_map
	)
	{
		std::vector<CDPoint_in> hole_points;
		std::map<int, std::pair<easy3d::vec3, easy3d::vec2>> vd_2d3d_map;
		project_hole_points(mesh, hole_edges, duplicate_vd_ind_map, hole_points, vd_2d3d_map);

		int pre_vs_ind = -1, pre_vt_ind = -1, first_vs_ind = -1, first_vt_ind = -1;
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		std::map<int, bool> visited_vd;
		bool complete_loop = false;
		for (int hpi = 0; hpi < hole_edges.size(); hpi++)
		{
			easy3d::SurfaceMesh::Vertex vs(hole_edges[hpi].first);
			easy3d::SurfaceMesh::Vertex vt(hole_edges[hpi].second);
			int vs_ind = vs.idx();
			int vt_ind = vt.idx();
			if (is_duplicated_vertex[vs])
				vs_ind = duplicate_vd_ind_map[vs];
			if (is_duplicated_vertex[vt])
				vt_ind = duplicate_vd_ind_map[vt];

			easy3d::vec2 p_vs_2d = vd_2d3d_map[vs_ind].second;
			easy3d::vec2 p_vt_2d = vd_2d3d_map[vt_ind].second;

			if (hpi == 0)
			{
				polygon2d_constraints.push_back(CDPoint_in(p_vs_2d.x, p_vs_2d.y));
				polygon2d_constraints.push_back(CDPoint_in(p_vt_2d.x, p_vt_2d.y));
				first_vs_ind = vs_ind;
				first_vt_ind = vt_ind;


				if (visited_vd.find(vs_ind) == visited_vd.end())
				{
					visited_vd[vs_ind] = true;
					poly_points_pairs.push_back(std::make_pair(CDPoint_in(p_vs_2d.x, p_vs_2d.y), poly_points_pairs.size()));
					poly_index_map[poly_points_pairs.size() - 1] = vs.idx();
				}

				if (visited_vd.find(vt_ind) == visited_vd.end())
				{
					visited_vd[vt_ind] = true;
					poly_points_pairs.push_back(std::make_pair(CDPoint_in(p_vt_2d.x, p_vt_2d.y), poly_points_pairs.size()));
					poly_index_map[poly_points_pairs.size() - 1] = vt.idx();
				}
			}
			else
			{
				if (vs_ind != pre_vs_ind && vs_ind != pre_vt_ind && (vt_ind == pre_vs_ind || vt_ind == pre_vt_ind))
				{
					polygon2d_constraints.push_back(CDPoint_in(p_vs_2d.x, p_vs_2d.y));
					if (vs_ind == first_vs_ind || vs_ind == first_vt_ind)
						complete_loop = true;

					if (visited_vd.find(vs_ind) == visited_vd.end())
					{
						visited_vd[vs_ind] = true;
						poly_points_pairs.push_back(std::make_pair(CDPoint_in(p_vs_2d.x, p_vs_2d.y), poly_points_pairs.size()));
						poly_index_map[poly_points_pairs.size() - 1] = vs.idx();
					}
				}
				else if (vt_ind != pre_vs_ind && vt_ind != pre_vt_ind && (vs_ind == pre_vs_ind || vs_ind == pre_vt_ind))
				{
					polygon2d_constraints.push_back(CDPoint_in(p_vt_2d.x, p_vt_2d.y));
					if (vt_ind == first_vs_ind || vt_ind == first_vt_ind)
						complete_loop = true;
					if (visited_vd.find(vt_ind) == visited_vd.end())
					{
						visited_vd[vt_ind] = true;
						poly_points_pairs.push_back(std::make_pair(CDPoint_in(p_vt_2d.x, p_vt_2d.y), poly_points_pairs.size()));
						poly_index_map[poly_points_pairs.size() - 1] = vt.idx();
					}
				}
			}

			pre_vs_ind = vs_ind;
			pre_vt_ind = vt_ind;

			if (complete_loop)
				break;
		}

		if (!complete_loop)
		{
			polygon2d_constraints.push_back(polygon2d_constraints[0]);
			return false;
		}
		else
		{
			return true;
		}

		//{		
		//	// To visiualize for testing
		//  auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		//	auto graph = new easy3d::Graph;
		//	for (int pi = 0; pi < poly_points_pairs.size(); ++pi)
		//	{
		//		easy3d::SurfaceMesh::Vertex vd(poly_index_map[pi]);
		//		graph->add_vertex(get_points_coord[vd]);
		//		if (pi > 0)
		//		{
		//			easy3d::Graph::Vertex vsg(pi - 1), vtg(pi);
		//			graph->add_edge(vsg, vtg);
		//		}
		//	}
		//	easy3d::Graph::Vertex vsg(poly_points_pairs.size() - 1), vtg(0);
		//	graph->add_edge(vsg, vtg);

		//	easy3d::Viewer viewer("test");
		//	viewer.camera()->setViewDirection(easy3d::vec3(0, 0, -1));
		//	viewer.camera()->setUpVector(easy3d::vec3(0, 1, 0));
		//	viewer.add_model(graph);
		//	viewer.run();
		//}

	}

	void hole_surface_recosntruction_cgal_constrained_delaunay
	(
		easy3d::SurfaceMesh* mesh,
		Polygon_2_in& polygon2d_constraints,
		std::vector<std::pair<CDPoint_in, unsigned>>& poly_points_pairs,
		std::map<int, int>& poly_index_map,
		std::vector<Facet>& facets_dt,
		const bool is_a_loop,
		const int h_i
	)
	{
		//Insert the polygons into a constrained triangulation
		CDT_in cdt;
		cdt.insert(poly_points_pairs.begin(), poly_points_pairs.end());
		if (is_a_loop)
			cdt.insert_constraint(polygon2d_constraints.begin(), polygon2d_constraints.end(), is_a_loop);

		if (!cdt.finite_face_handles().empty())
		{
			std::unordered_map<Face_handle_in, bool> in_domain_map;
			boost::associative_property_map<std::unordered_map<Face_handle_in, bool>> in_domain(in_domain_map);

			//Mark facets that are inside the domain bounded by the polygon
			if (is_a_loop)
				CGAL::mark_domain_in_triangulation(cdt, in_domain);

			for (Face_handle_in f : cdt.finite_face_handles())
			{
				int v0_ind = (*f).vertex(0)->info();
				int v1_ind = (*f).vertex(1)->info();
				int v2_ind = (*f).vertex(2)->info();

				if (v0_ind < poly_points_pairs.size() && v0_ind > -1
					&& v1_ind < poly_points_pairs.size() && v1_ind > -1
					&& v2_ind < poly_points_pairs.size() && v2_ind > -1
					&& v0_ind != v1_ind && v0_ind != v2_ind && v1_ind != v2_ind)
				{
					Facet fd_dt = { poly_index_map[v0_ind], poly_index_map[v1_ind], poly_index_map[v2_ind] };
					if ((is_a_loop && get(in_domain, f)) || !is_a_loop)
					{
						facets_dt.push_back(fd_dt);
					}
				}
			}
		}
	}

	void triangulate_holes_without_self_intersection
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		easy3d::SurfaceMeshBuilder& builder,
		std::vector<Facet>& facets_dt
	)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		auto is_duplicated_vertex = mesh->get_vertex_property<bool>("v:is_duplicated_vertex");
		auto duplicated_vertex_ids = mesh->get_vertex_property<std::set<int>>("v:duplicate_vertex_ids");
		auto fd_plane = mesh->get_face_property<easy3d::Plane3>("f:face_plane");
		auto fd_center = mesh->get_face_property<easy3d::vec3>("f:face_center");
		auto fd_cgal_tri2 = mesh->get_face_property<Triangle_2>("f:face_cgal_tri2");
		for (auto& f_cgal : facets_dt)
		{
			easy3d::SurfaceMesh::Vertex v0(f_cgal[0]);
			easy3d::SurfaceMesh::Vertex v1(f_cgal[1]);
			easy3d::SurfaceMesh::Vertex v2(f_cgal[2]);
			easy3d::vec3 p0 = get_points_coord[v0];
			easy3d::vec3 p1 = get_points_coord[v1];
			easy3d::vec3 p2 = get_points_coord[v2];

			std::set<int> face_verts_ids;
			face_verts_ids.insert(v0.idx()); face_verts_ids.insert(v1.idx()); face_verts_ids.insert(v2.idx());
			if (is_duplicated_vertex[v0])
				face_verts_ids.insert(duplicated_vertex_ids[v0].begin(), duplicated_vertex_ids[v0].end());
			if (is_duplicated_vertex[v1])
				face_verts_ids.insert(duplicated_vertex_ids[v1].begin(), duplicated_vertex_ids[v1].end());
			if (is_duplicated_vertex[v2])
				face_verts_ids.insert(duplicated_vertex_ids[v2].begin(), duplicated_vertex_ids[v2].end());

			bool is_self_intersect = false;
			//self_intersection detection
			for (auto& v_id : face_verts_ids)
			{
				easy3d::SurfaceMesh::Vertex vd(v_id);
				for (auto& f_neg : mesh->faces(vd))
				{
					std::map<easy3d::SurfaceMesh::Vertex, bool> f_neg_vi_t_map;
					for (auto f_neg_vd : mesh->vertices(f_neg))
						f_neg_vi_t_map[f_neg_vd] = false;
					easy3d::SurfaceMesh::Vertex fc_vi_nt, f_neg_vi_nt;
					std::map<int, int> local_visited_neg_vd;
					int count_equal_vd = 0, pre_count_equal_vd = 0;
					for (auto fc_v_id : face_verts_ids)
					{
						easy3d::SurfaceMesh::Vertex fc_vd(fc_v_id);
						pre_count_equal_vd = count_equal_vd;
						int fc_vind = -1;
						if (is_duplicated_vertex[fc_vd])
							fc_vind = duplicate_vd_ind_map[fc_vd];
						else
							fc_vind = fc_vd.idx();

						for (auto f_neg_vd : mesh->vertices(f_neg))
						{
							if (!f_neg_vi_t_map[f_neg_vd])
							{
								int f_neg_vind = -1;
								if (is_duplicated_vertex[f_neg_vd])
									f_neg_vind = duplicate_vd_ind_map[f_neg_vd];
								else
									f_neg_vind = f_neg_vd.idx();

								if (fc_vind == f_neg_vind)
								{
									f_neg_vi_t_map[f_neg_vd] = true;
									++count_equal_vd;
									break;
								}
							}
						}

						//get non-touched vertex
						if (count_equal_vd == pre_count_equal_vd)
							fc_vi_nt = fc_vd;
					}

					if (count_equal_vd == 2)
					{
						//detecting self-intersected facets
						for (auto f1_vd : mesh->vertices(f_neg))
						{
							if (!f_neg_vi_t_map[f1_vd])
							{
								f_neg_vi_nt = f1_vd;
								break;
							}
						}

						double v0_dis = std::sqrt(fd_plane[f_neg].squared_distance(p0));
						double v1_dis = std::sqrt(fd_plane[f_neg].squared_distance(p1));
						double v2_dis = std::sqrt(fd_plane[f_neg].squared_distance(p2));
						double max_dis2plane = std::max(std::max(v0_dis, v1_dis), v2_dis);

						Polygon_2 intersected_poly_fc_f_neg;
						Triangle_2 f_neg_tri2 = fd_cgal_tri2[f_neg];
						easy3d::Plane3 plane_f_neg = mesh->get_face_property<easy3d::Plane3>("f:face_plane")[f_neg];
						auto p0_proj = to_2d_new(plane_f_neg, plane_point(plane_f_neg), p0);
						auto p1_proj = to_2d_new(plane_f_neg, plane_point(plane_f_neg), p1);
						auto p2_proj = to_2d_new(plane_f_neg, plane_point(plane_f_neg), p2);
						Triangle_2 fc_f_neg_proj_tri2(Point_2(p0_proj.x, p0_proj.y), Point_2(p1_proj.x, p1_proj.y), Point_2(p2_proj.x, p2_proj.y));
						triangle_2D_intersection_test(f_neg_tri2, fc_f_neg_proj_tri2, intersected_poly_fc_f_neg);
						double intersect_poly_area_fc_f_neg = std::abs(CGAL::to_double(intersected_poly_fc_f_neg.area()));
						double fc_f_neg_area_ratio = intersect_poly_area_fc_f_neg / std::abs(CGAL::to_double(f_neg_tri2.area()));

						if (max_dis2plane < point_to_plane_dis_thres && fc_f_neg_area_ratio > intersected_area_ratio)
						{
							is_self_intersect = true;
							break;
						}
					}
					else if (count_equal_vd == 3)
					{
						is_self_intersect = true;
						break;
					}
				}

				if (is_self_intersect)
					break;
			}

			if (!is_self_intersect)
			{
				builder.add_triangle(v0, v1, v2);
			}
		}
	}

	void remesh_holes
	(
		easy3d::SurfaceMesh* mesh,
		std::map<easy3d::SurfaceMesh::Vertex, int>& duplicate_vd_ind_map,
		std::vector<std::vector <std::pair<int, int>>>& holes_edges,
		int& num_closed
	)
	{
		std::vector<Facet> facets_dt;
		for (int h_i = 0; h_i < holes_edges.size(); ++h_i)
		{
			int pre_facets_dt_size = facets_dt.size();
			Polygon_2_in polygon2d;
			std::map<int, std::pair<easy3d::vec3, easy3d::vec2>> vd_2d3d_map;
			std::vector< std::pair<CDPoint_in, unsigned> > poly_points_pairs;
			std::map<int, int> poly_index_map;
			bool is_a_loop = extract_polyline_from_holes(mesh, holes_edges[h_i], duplicate_vd_ind_map, polygon2d, poly_points_pairs, poly_index_map);
			hole_surface_recosntruction_cgal_constrained_delaunay(mesh, polygon2d, poly_points_pairs, poly_index_map, facets_dt, is_a_loop, h_i);
			if (facets_dt.size() > pre_facets_dt_size)
				++num_closed;
		}

		// close large holes by rebuild mesh
		easy3d::SurfaceMesh mesh_clone;
		easy3d::SurfaceMeshBuilder builder(&mesh_clone);
		builder.begin_surface();
		copy_mesh(mesh, builder);

		// mesh holes
		triangulate_holes_without_self_intersection(mesh, duplicate_vd_ind_map, builder, facets_dt);

		builder.end_surface(true);
		mesh->clear();
		*mesh = mesh_clone;
	}

	void advacing_holes_filling(easy3d::SurfaceMesh* mesh) //for large holes
	{
		easy3d::StopWatch w;
		w.start();
		LOG(INFO) << "	- advance fill holes ... ";
		std::map<easy3d::SurfaceMesh::Vertex, int> duplicate_vd_ind_map;
		holes_filling_preprocessing(mesh, duplicate_vd_ind_map);

		int min_hole_size = easy3d::max<int>(), max_hole_size = -easy3d::max<int>();
		std::vector<std::vector <std::pair<int, int>>> holes_edges;
		complex_holes_detection(mesh, duplicate_vd_ind_map, holes_edges, min_hole_size, max_hole_size);

		int num_closed = 0;
		if (!holes_edges.empty())
			remesh_holes(mesh, duplicate_vd_ind_map, holes_edges, num_closed);
		holes_filling_postprocessing(mesh);

		if (holes_edges.empty())
		{
			if (min_hole_size == easy3d::max<int>() && max_hole_size == -easy3d::max<int>())
				LOG(WARNING) << "model is closed and no holes to fill";
			else
				LOG(WARNING) << "no holes meet the requirement (smallest: " << min_hole_size << ", largest: " << max_hole_size << ")";
		}
		else
		{
			LOG(INFO) << num_closed <<" / " << holes_edges.size() << " holes are filled" << ", done in (s): " << w.time_string();
		}
	}
}