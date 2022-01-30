/*
 * Copyright 2011-2013 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Source: blender/intern/cycles/blender/blender_mesh.cpp

#include "util_raytracing/mesh.hpp"
#include "unirender/cycles/renderer.hpp"
#include <scene/mesh.h>
#include <mikktspace.h>
#if 0
struct MikkUserData {
  MikkUserData(
               const char *layer_name,
               const ccl::Mesh *mesh,
               ccl::float3 *tangent,
               float *tangent_sign)
      : mesh(mesh), texface(NULL), orco(NULL), tangent(tangent), tangent_sign(tangent_sign)
  {
    const ccl::AttributeSet &attributes = (mesh->subd_faces.size()) ? mesh->subd_attributes :
                                                                 mesh->attributes;

    ccl::Attribute *attr_vN = attributes.find(ccl::ATTR_STD_VERTEX_NORMAL);
    vertex_normal = attr_vN->data_float3();

    if (layer_name == NULL) {
      ccl::Attribute *attr_orco = attributes.find(ccl::ATTR_STD_GENERATED);

      if (attr_orco) {
        orco = attr_orco->data_float3();
       // mesh_texture_space(*(BL::Mesh *)&b_mesh, orco_loc, orco_size);
      }
    }
    else {
      ccl::Attribute *attr_uv = attributes.find(ccl::ustring(layer_name));
      if (attr_uv != NULL) {
        texface = attr_uv->data_float2();
      }
    }
  }

  const ccl::Mesh *mesh;
  int num_faces;

  ccl::float3 *vertex_normal;
  ccl::float2 *texface;
  ccl::float3 *orco;
  ccl::float3 orco_loc, orco_size;

  ccl::float3 *tangent;
  float *tangent_sign;
};

static int mikk_get_num_faces(const SMikkTSpaceContext *context)
{
  const MikkUserData *userdata = (const MikkUserData *)context->m_pUserData;
  if (userdata->mesh->subd_faces.size()) {
    return userdata->mesh->subd_faces.size();
  }
  else {
    return userdata->mesh->num_triangles();
  }
}

static int mikk_get_num_verts_of_face(const SMikkTSpaceContext *context, const int face_num)
{
  const MikkUserData *userdata = (const MikkUserData *)context->m_pUserData;
  if (userdata->mesh->subd_faces.size()) {
    const ccl::Mesh *mesh = userdata->mesh;
    return mesh->subd_faces[face_num].num_corners;
  }
  else {
    return 3;
  }
}

static int mikk_vertex_index(const ccl::Mesh *mesh, const int face_num, const int vert_num)
{
  if (mesh->subd_faces.size()) {
    const ccl::Mesh::SubdFace &face = mesh->subd_faces[face_num];
    return mesh->subd_face_corners[face.start_corner + vert_num];
  }
  else {
    return mesh->triangles[face_num * 3 + vert_num];
  }
}

static int mikk_corner_index(const ccl::Mesh *mesh, const int face_num, const int vert_num)
{
  if (mesh->subd_faces.size()) {
    const ccl::Mesh::SubdFace &face = mesh->subd_faces[face_num];
    return face.start_corner + vert_num;
  }
  else {
    return face_num * 3 + vert_num;
  }
}

static void mikk_get_position(const SMikkTSpaceContext *context,
                              float P[3],
                              const int face_num,
                              const int vert_num)
{
  const MikkUserData *userdata = (const MikkUserData *)context->m_pUserData;
  const ccl::Mesh *mesh = userdata->mesh;
  const int vertex_index = mikk_vertex_index(mesh, face_num, vert_num);
  const ccl::float3 vP = mesh->verts[vertex_index];
  P[0] = vP.x;
  P[1] = vP.y;
  P[2] = vP.z;
}

static void mikk_get_texture_coordinate(const SMikkTSpaceContext *context,
                                        float uv[2],
                                        const int face_num,
                                        const int vert_num)
{
  const MikkUserData *userdata = (const MikkUserData *)context->m_pUserData;
  const ccl::Mesh *mesh = userdata->mesh;
  if (userdata->texface != NULL) {
    const int corner_index = mikk_corner_index(mesh, face_num, vert_num);
    ccl::float2 tfuv = userdata->texface[corner_index];
    uv[0] = tfuv.x;
    uv[1] = tfuv.y;
  }
  else if (userdata->orco != NULL) {
    const int vertex_index = mikk_vertex_index(mesh, face_num, vert_num);
    const ccl::float3 orco_loc = userdata->orco_loc;
    const ccl::float3 orco_size = userdata->orco_size;
    const ccl::float3 orco = (userdata->orco[vertex_index] + orco_loc) / orco_size;

    const ccl::float2 tmp = map_to_sphere(orco);
    uv[0] = tmp.x;
    uv[1] = tmp.y;
  }
  else {
    uv[0] = 0.0f;
    uv[1] = 0.0f;
  }
}

static void mikk_get_normal(const SMikkTSpaceContext *context,
                            float N[3],
                            const int face_num,
                            const int vert_num)
{
  const MikkUserData *userdata = (const MikkUserData *)context->m_pUserData;
  const ccl::Mesh *mesh = userdata->mesh;
  ccl::float3 vN;
  if (mesh->subd_faces.size()) {
    const ccl::Mesh::SubdFace &face = mesh->subd_faces[face_num];
    if (face.smooth) {
      const int vertex_index = mikk_vertex_index(mesh, face_num, vert_num);
      vN = userdata->vertex_normal[vertex_index];
    }
    else {
      vN = face.normal(mesh);
    }
  }
  else {
    if (mesh->smooth[face_num]) {
      const int vertex_index = mikk_vertex_index(mesh, face_num, vert_num);
      vN = userdata->vertex_normal[vertex_index];
    }
    else {
      const ccl::Mesh::Triangle tri = mesh->get_triangle(face_num);
      vN = tri.compute_normal(&mesh->verts[0]);
    }
  }
  N[0] = vN.x;
  N[1] = vN.y;
  N[2] = vN.z;
}

static void mikk_set_tangent_space(const SMikkTSpaceContext *context,
                                   const float T[],
                                   const float sign,
                                   const int face_num,
                                   const int vert_num)
{
  MikkUserData *userdata = (MikkUserData *)context->m_pUserData;
  const ccl::Mesh *mesh = userdata->mesh;
  const int corner_index = mikk_corner_index(mesh, face_num, vert_num);
  userdata->tangent[corner_index] = ccl::make_float3(T[0], T[1], T[2]);
  if (userdata->tangent_sign != NULL) {
    userdata->tangent_sign[corner_index] = sign;
  }
}
#endif
void unirender::cycles::compute_tangents(
    ccl::Mesh *mesh, bool need_sign, bool active_render)
{
#if 0
    const char *layer_name = nullptr;
  /* Create tangent attributes. */
  ccl::AttributeSet &attributes = (mesh->subd_faces.size()) ? mesh->subd_attributes : mesh->attributes;
  ccl::Attribute *attr;
  ccl::ustring name;
  if (layer_name != NULL) {
    name = ccl::ustring((ccl::string(layer_name) + ".tangent").c_str());
  }
  else {
    name = ccl::ustring("orco.tangent");
  }
  if (active_render) {
    //attr = attributes.add(ccl::ATTR_STD_UV_TANGENT, name);
	  attr = attributes.find(ccl::ATTR_STD_UV_TANGENT);
  }
  else {
    attr = attributes.add(name, ccl::TypeDesc::TypeVector, ccl::ATTR_ELEMENT_CORNER);
  }
  ccl::float3 *tangent = attr->data_float3();
  /* Create bitangent sign attribute. */
  float *tangent_sign = NULL;
  if (need_sign) {
    ccl::Attribute *attr_sign;
    ccl::ustring name_sign;
    if (layer_name != NULL) {
      name_sign = ccl::ustring((ccl::string(layer_name) + ".tangent_sign").c_str());
    }
    else {
      name_sign = ccl::ustring("orco.tangent_sign");
    }

    if (active_render) {
      //attr_sign = attributes.add(ccl::ATTR_STD_UV_TANGENT_SIGN, name_sign);
      attr_sign = attributes.find(ccl::ATTR_STD_UV_TANGENT_SIGN);
    }
    else {
      attr_sign = attributes.add(name_sign, ccl::TypeDesc::TypeFloat, ccl::ATTR_ELEMENT_CORNER);
    }
    tangent_sign = attr_sign->data_float();
  }
  /* Setup userdata. */
  MikkUserData userdata(layer_name, mesh, tangent, tangent_sign);
  /* Setup interface. */
  SMikkTSpaceInterface sm_interface;
  memset(&sm_interface, 0, sizeof(sm_interface));
  sm_interface.m_getNumFaces = mikk_get_num_faces;
  sm_interface.m_getNumVerticesOfFace = mikk_get_num_verts_of_face;
  sm_interface.m_getPosition = mikk_get_position;
  sm_interface.m_getTexCoord = mikk_get_texture_coordinate;
  sm_interface.m_getNormal = mikk_get_normal;
  sm_interface.m_setTSpaceBasic = mikk_set_tangent_space;
  /* Setup context. */
  SMikkTSpaceContext context;
  memset(&context, 0, sizeof(context));
  context.m_pUserData = &userdata;
  context.m_pInterface = &sm_interface;
  /* Compute tangents. */
  genTangSpaceDefault(&context);
#endif
}
