#include "framefab_task_sequence_planner/utils/WireFrame.h"
#include <ros/console.h>

WireFrame::WireFrame()
    :delta_tol_(1e-1), unify_size_(2.0), layer_size_(0), unit_scale_(1.0)
{
  pvert_list_ = new vector<WF_vert*>;
  pedge_list_ = new vector<WF_edge*>;
}


WireFrame::~WireFrame()
{
  int N = pvert_list_->size();
  for (int i = 0; i < N; i++)
  {
    delete (*pvert_list_)[i];
    (*pvert_list_)[i] = NULL;
  }
  delete pvert_list_;
  pvert_list_ = NULL;

  int M = pedge_list_->size();
  for (int i = 0; i < M; i++)
  {
    delete (*pedge_list_)[i];
    (*pedge_list_)[i] = NULL;
  }
  delete pedge_list_;
  pedge_list_ = NULL;
}


void WireFrame::LoadFromOBJ(const char *path)
{
  /* need to deal with replication */

  FILE *fp = fopen(path, "r");

  try
  {
    // read vertexes
    fseek(fp, 0, SEEK_SET);
    char pLine[512];
    char *tok;
    vector<WF_vert*> tmp_points;
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'v' && pLine[1] == ' ')
      {
        Vec3f p;
        char tmp[128];
        tok = strtok(pLine, " ");
        for (int i = 0; i < 3; i++)
        {
          tok = strtok(NULL, " ");
          strcpy(tmp, tok);
          tmp[strcspn(tmp, " ")] = 0;
          p[i] = (float)atof(tmp);
        }

        p = point(p.x(), p.y(), p.z());
        WF_vert *u = InsertVertex(p);
        tmp_points.push_back(u);
      }
    }

    // read lines
    char c;
    int prev;
    int curv;
    fseek(fp, 0, SEEK_SET);
    while (c = fgetc(fp), c != EOF)
    {
      while (c != 'l' && c != EOF)
      {
        c = fgetc(fp);
      }

      if (c == '\n' || c == EOF || (c = fgetc(fp)) != ' ')
      {
        continue;
      }

      prev = -1;
      while (c != '\n' && c != EOF)
      {
        while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
          ;

        if (c == '\n' || c == EOF)
        {
          break;
        }

        for (curv = 0; isdigit(c); c = fgetc(fp))
        {
          curv = curv * 10 + c - '0';
        }
        curv--;

        if (prev != -1)
        {
          InsertEdge(tmp_points[prev], tmp_points[curv]);
        }

        prev = curv;
      }
    }

    // read faces
    fseek(fp, 0, SEEK_SET);
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'f' && pLine[1] == ' ')
      {
        vector<WF_vert*> bound_points;
        tok = strtok(pLine, " ");
        char tmp[128];
        while (tok = strtok(NULL, " "))
        {
          strcpy(tmp, tok);
          tmp[strcspn(tmp, " ")] = 0;
          int u = (int)atof(tmp) - 1;
          bound_points.push_back(tmp_points[u]);
        }

        int Bn = bound_points.size();
        for (int i = 0; i < Bn - 1; i++)
        {
          InsertEdge(bound_points[i], bound_points[i + 1]);
        }
        InsertEdge(bound_points[Bn - 1], bound_points[0]);
      }
    }

    Unify();
  }
  catch (...)
  {
    return;
  }

  fclose(fp);
}


void WireFrame::LoadFromPWF(const char *path)
{
  /* assert that there is no replication in PWF */

  FILE *fp = fopen(path, "r");

  try
  {
    // read vertexes
    fseek(fp, 0, SEEK_SET);
    char pLine[512];
    char *tok;
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'v' && pLine[1] == ' ')
      {
        Vec3f p;
        char tmp[128];
        tok = strtok(pLine, " ");
        for (int i = 0; i < 3; i++)
        {
          tok = strtok(NULL, " ");
          strcpy(tmp, tok);
          tmp[strcspn(tmp, " ")] = 0;
          p[i] = (float)atof(tmp);
        }

        p = point(p.x(), p.y(), p.z());
        InsertVertex(p);
      }
    }

    // read layer
    fseek(fp, 0, SEEK_SET);
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'g' && pLine[1] == ' ')
      {
        tok = strtok(pLine, " ");

        char tmp[128];
        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int u = (int)atof(tmp) - 1;

        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int v = (int)atof(tmp) - 1;

        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int layer = (int)atof(tmp);

        WF_edge *e = InsertEdge((*pvert_list_)[u], (*pvert_list_)[v]);
        if (e != NULL)
        {
          if(e->Layer() != -1)
          {
            ROS_WARN_STREAM("Overwrite previously set id! - prev layer id : " << e->Layer());
            assert(e->Layer() == -1);
          }

          e->SetLayer(layer);
          e->ppair_->SetLayer(layer);
        }
      }
    }

    // read lines
    char c;
    int prev;
    int curv;
    fseek(fp, 0, SEEK_SET);
    while (c = fgetc(fp), c != EOF)
    {
      while (c != 'l' && c != EOF)
      {
        c = fgetc(fp);
      }

      if (c == '\n' || c == EOF || (c = fgetc(fp)) != ' ')
      {
        continue;
      }

      prev = -1;
      while (c != '\n' && c != EOF)
      {
        while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
          ;

        if (c == '\n' || c == EOF)
        {
          break;
        }

        for (curv = 0; isdigit(c); c = fgetc(fp))
        {
          curv = curv * 10 + c - '0';
        }
        curv--;

        if (prev != -1)
        {
          InsertEdge((*pvert_list_)[prev], (*pvert_list_)[curv]);
        }

        prev = curv;
      }
    }

    // read pillar
    fseek(fp, 0, SEEK_SET);
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'p' && pLine[1] == ' ')
      {
        tok = strtok(pLine, " ");

        char tmp[128];
        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int u = (int)atof(tmp) - 1;

        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int v = (int)atof(tmp) - 1;

        WF_vert *b = (*pvert_list_)[u];
        b->SetBase(true);

        WF_vert *f = (*pvert_list_)[v];
        f->SetFixed(true);

        WF_edge *e = InsertEdge(b, f);
        if (e != NULL)
        {
          e->SetPillar(true);
          e->ppair_->SetPillar(true);
        }
      }
    }

    // read ceiling
    fseek(fp, 0, SEEK_SET);
    while (fgets(pLine, 512, fp))
    {
      if (pLine[0] == 'c' && pLine[1] == ' ')
      {
        tok = strtok(pLine, " ");

        char tmp[128];
        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int u = (int)atof(tmp) - 1;

        tok = strtok(NULL, " ");
        strcpy(tmp, tok);
        tmp[strcspn(tmp, " ")] = 0;
        int v = (int)atof(tmp) - 1;

        WF_edge *e = InsertEdge((*pvert_list_)[u], (*pvert_list_)[v]);
        if (e != NULL)
        {
          e->SetCeiling(true);
          e->ppair_->SetCeiling(true);
        }
      }
    }

    Unify();
  }
  catch (...)
  {
    return;
  }

  fclose(fp);
}


void WireFrame::WriteToOBJ(const char *path)
{
  FILE *fp = fopen(path, "wb+");
  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  vector<int> export_vert;
  vector<int> hash(N);
  fill(hash.begin(), hash.end(), -1);

  for (int i = 0; i < N; i++)
  {
    if (!isFixed(i))
    {
      export_vert.push_back(i);
      hash[i] = export_vert.size();
    }
  }

  int Ne = export_vert.size();
  for (int i = 0; i < Ne; i++)
  {
    point p = (*pvert_list_)[export_vert[i]]->Position();
    fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
  }

  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && !e1->isPillar())
    {
      int u = e2->pvert_->ID();
      int v = e1->pvert_->ID();
      fprintf(fp, "l %d %d\n", hash[u], hash[v]);
    }
  }

  fclose(fp);
}


void WireFrame::WriteToPWF(
    bool bVert, bool bLine,
    bool bPillar, bool bCeiling,
    bool bCut, int min_layer, int max_layer,
    const char *path)
{
  if ((*pedge_list_)[0]->Layer() == -1 || !bCut)
  {
    min_layer = -(1 << 20);
    max_layer = (1 << 20);
  }


  FILE *fp = fopen(path, "wb+");
  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  vector<int> export_vert;
  vector<int> hash(N);
  fill(hash.begin(), hash.end(), -1);

  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
    {
      if (e1->isPillar() && !bPillar)
      {
        continue;
      }

      int u = e2->pvert_->ID();
      int v = e1->pvert_->ID();
      if (hash[u] == -1)
      {
        export_vert.push_back(u);
        hash[u] = export_vert.size();
      }
      if (hash[v] == -1)
      {
        export_vert.push_back(v);
        hash[v] = export_vert.size();
      }
    }
  }

  int Ne = export_vert.size();
  if (bVert)
  {
    for (int i = 0; i < Ne; i++)
    {
      point p = (*pvert_list_)[export_vert[i]]->Position();
      fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
    }
  }

  if (bLine)
  {
    for (int i = 0; i < M; i++)
    {
      WF_edge *e1 = (*pedge_list_)[i];
      WF_edge *e2 = e1->ppair_;
      if (i < e2->ID() && !e1->isPillar() &&
          e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
      {
        int u = e2->pvert_->ID();
        int v = e1->pvert_->ID();
        fprintf(fp, "l %d %d\n", hash[u], hash[v]);
      }
    }
  }

  if (bPillar)
  {
    for (int i = 0; i < M; i++)
    {
      WF_edge *e1 = (*pedge_list_)[i];
      WF_edge *e2 = e1->ppair_;
      if (i < e2->ID() && e1->isPillar()
          && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
      {
        int u = e2->pvert_->ID();
        int v = e1->pvert_->ID();
        assert(e2->pvert_->isBase());
        fprintf(fp, "p %d %d\n", hash[u], hash[v]);
      }
    }
  }

  if (bCeiling)
  {
    for (int i = 0; i < M; i++)
    {
      WF_edge *e1 = (*pedge_list_)[i];
      WF_edge *e2 = e1->ppair_;
      if (i < e2->ID() && e1->isCeiling()
          && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
      {
        int u = e2->pvert_->ID();
        int v = e1->pvert_->ID();
        fprintf(fp, "c %d %d\n", hash[u], hash[v]);
      }
    }
  }

  if (bCut)
  {
    for (int i = 0; i < M; i++)
    {
      WF_edge *e1 = (*pedge_list_)[i];
      WF_edge *e2 = e1->ppair_;
      if (i < e2->ID() && e1->Layer() != -1
          && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
      {
        int u = e2->pvert_->ID();
        int v = e1->pvert_->ID();
        fprintf(fp, "g %d %d %d\n", hash[u], hash[v], e1->Layer());
      }
    }
  }

  fclose(fp);
}


void WireFrame::ImportFrom3DD(const char *path)
{
  FILE *fp = fopen(path, "r");

  try
  {
    // read vertexes
    fseek(fp, 0, SEEK_SET);
    char pLine[512];
    char *tok;
    while (fgets(pLine, 512, fp))
    {
      string flag = "";
      for (int i = 2; i < 9; i++)
      {
        flag = flag + pLine[i];
      }
      if (pLine[0] == '#' && flag == "element")
      {
        WF_vert *prev = NULL;
        while (fgets(pLine, 512, fp) && pLine[0] == ' ')
        {
          Vec3f p;
          char tmp[128];

          tok = strtok(pLine, " ");
          strcpy(tmp, tok);
          tmp[strcspn(tmp, " ")] = 0;
          p[0] = (float)atof(tmp);

          for (int i = 1; i < 3; i++)
          {
            tok = strtok(NULL, " ");
            strcpy(tmp, tok);
            tmp[strcspn(tmp, " ")] = 0;
            p[i] = (float)atof(tmp);
          }

          p = point(p.x(), p.y(), p.z());
          WF_vert *u = InsertVertex(p);
          if (prev != NULL)
          {
            InsertEdge(prev, u);
          }
          prev = u;
        }
      }
    }

    Unify();
  }
  catch (...)
  {
    return;
  }

  fclose(fp);
}


void WireFrame::ExportSubgraph(const char *path)
{
  FILE *fp = fopen(path, "wb+");
  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  vector<int> export_vert;
  vector<int> hash(N);
  fill(hash.begin(), hash.end(), -1);

  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && e1->isSubgraph())
    {
      int u = e2->pvert_->ID();
      int v = e1->pvert_->ID();
      if (hash[u] == -1)
      {
        export_vert.push_back(u);
        hash[u] = export_vert.size();
      }
      if (hash[v] == -1)
      {
        export_vert.push_back(v);
        hash[v] = export_vert.size();
      }
    }
  }

  int Ne = export_vert.size();
  for (int i = 0; i < Ne; i++)
  {
    point p = (*pvert_list_)[export_vert[i]]->Position();
    fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
  }

  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && e1->isSubgraph())
    {
      int u = e2->pvert_->ID();
      int v = e1->pvert_->ID();
      fprintf(fp, "l %d %d\n", hash[u], hash[v]);
    }
  }

  fclose(fp);
}


void WireFrame::ExportPoints(int min_layer, int max_layer, const char *path)
{
  if ((*pedge_list_)[0]->Layer() == -1)
  {
    min_layer = -(1 << 20);
    max_layer = (1 << 20);
  }

  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  vector<bool> is_export_vert(N);
  fill(is_export_vert.begin(), is_export_vert.end(), false);

  FILE *fp = fopen(path, "wb+");
  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
    {
      is_export_vert[e2->pvert_->ID()] = true;
      is_export_vert[e1->pvert_->ID()] = true;
    }
  }

  for (int i = 0; i < N; i++)
  {
    if (is_export_vert[i])
    {
      //point u = (*pvert_list_)[i]->Position();
      //fprintf(fp, "%lf %lf %lf\n", u.x() / 125.0, u.y() / 125.0, u.z() / 125.0);

      point u = (*pvert_list_)[i]->RenderPos();
      fprintf(fp, "%lf %lf %lf\r\n", u.x(), u.y(), u.z());
    }
  }

  fclose(fp);
}


void WireFrame::ExportLines(int min_layer, int max_layer, const char *path)
{
  if ((*pedge_list_)[0]->Layer() == -1)
  {
    min_layer = -(1 << 20);
    max_layer = (1 << 20);
  }

  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  FILE *fp = fopen(path, "wb+");
  for (int i = 0; i < M; i++)
  {
    WF_edge *e1 = (*pedge_list_)[i];
    WF_edge *e2 = e1->ppair_;
    if (i < e2->ID() && e1->Layer() >= min_layer - 1 && e1->Layer() < max_layer)
    {
      //point u = e2->pvert_->Position();
      //point v = e1->pvert_->Position();
      //fprintf(fp, "%lf %lf %lf %lf %lf %lf\n",
      //	u.x() / 125.0, u.y() / 125.0, u.z() / 125.0,
      //	v.x() / 125.0, v.y() / 125.0, v.z() / 125.0);

      point u = e2->pvert_->RenderPos();
      point v = e1->pvert_->RenderPos();
      fprintf(fp, "%lf %lf %lf %lf %lf %lf\r\n",
              u.x(), u.y(), u.z(), v.x(), v.y(), v.z());
    }
  }

  fclose(fp);
}


WF_vert* WireFrame::InsertVertex(Vec3f p)
{
  // detect duplication
  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    if (Dist(p, (*pvert_list_)[i]->Position()) < 1e-3)
    {
      return ( *pvert_list_)[i];
    }
  }

  WF_vert *vert = new WF_vert(p);
  pvert_list_->push_back(vert);
  vert->SetID(N);
  return vert;
}


WF_edge* WireFrame::InsertEdge(WF_vert *u, WF_vert *v)
{
  // detect duplication
  WF_edge *e = u->pedge_;
  while (e != NULL)
  {
    if (e->pvert_ == v)
    {
      return e;
    }
    e = e->pnext_;
  }

  WF_edge *e1 = InsertOneWayEdge(u, v);
  WF_edge *e2 = InsertOneWayEdge(v, u);
  if (e1 != NULL)
  {
    e1->ppair_ = e2;
    e2->ppair_ = e1;
  }
  return e1;
}


WF_edge* WireFrame::InsertOneWayEdge(WF_vert *u, WF_vert *v)
{
  if (u == v)
  {
    return NULL;
  }

  WF_edge *edge = new WF_edge();
  edge->pvert_ = v;
  edge->pnext_ = u->pedge_;
  u->pedge_ = edge;
  u->IncreaseDegree();

  pedge_list_->push_back(edge);
  return edge;
}


void WireFrame::Unify()
{
  maxx_ = -1e20;
  maxy_ = -1e20;
  maxz_ = -1e20;
  minx_ = 1e20;
  miny_ = 1e20;
  minz_ = 1e20;
  basez_ = 1e20;

  fixed_vert_ = 0;
  base_vert_ = 0;
  pillar_size_ = 0;
  ceiling_size_ = 0;
  layer_size_ = 0;

  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    (*pvert_list_)[i]->SetID(i);
    point p = (*pvert_list_)[i]->Position();

    if (!(*pvert_list_)[i]->isFixed())
    {
      if (p.x() > maxx_)
      {
        maxx_ = p.x();
      }
      if (p.y() > maxy_)
      {
        maxy_ = p.y();
      }
      if (p.z() > maxz_)
      {
        maxz_ = p.z();
      }
      if (p.x() < minx_)
      {
        minx_ = p.x();
      }
      if (p.y() < miny_)
      {
        miny_ = p.y();
      }
      if (p.z() < minz_)
      {
        minz_ = p.z();
      }
    }

    //set base_z for once
    if (p.z() < basez_)
    {
      basez_ = p.z();
    }
  }

  int M = SizeOfEdgeList();
  for (int i = 0; i < M; i++)
  {
    (*pedge_list_)[i]->SetID(i);
    if ((*pedge_list_)[i]->isPillar())
    {
      pillar_size_++;
    }
    if ((*pedge_list_)[i]->isCeiling())
    {
      ceiling_size_++;
    }
    if ((*pedge_list_)[i]->Layer() > layer_size_)
    {
      layer_size_ = (*pedge_list_)[i]->Layer();
    }
  }

  if(pillar_size_ > 0)
  {
    // pillar is always in the first layer if exists
    if(0 == layer_size_)
    {
      layer_size_ = 1;
    }

    for (int i = 0; i < M; i++)
    {
      if (!(*pedge_list_)[i]->isPillar())
      {
        if(1 == layer_size_)
        {
          (*pedge_list_)[i]->SetLayer(1);
        }
        else
        {
          assert(0 != (*pedge_list_)[i]->Layer());
        }
      }
      else
      {
        (*pedge_list_)[i]->SetLayer(0);
      }
    }

    // add base
    layer_size_++;
  }

  float scaleX = maxx_ - minx_;
  float scaleY = maxy_ - miny_;
  float scaleZ = maxz_ - minz_;
  float scaleMax = scaleX;
  if (scaleMax < scaleY)
  {
    scaleMax = scaleY;
  }
  if (scaleMax < scaleZ)
  {
    scaleMax = scaleZ;
  }

  scaleV_ = unify_size_ / scaleMax;
  center_pos_ = point((minx_ + maxx_) / 2.f, (miny_ + maxy_) / 2.f, (minz_ + maxz_) / 2.f);
  base_center_pos_ = point((minx_ + maxx_) / 2.f, (miny_ + maxy_) / 2.f, basez_);

  for (size_t i = 0; i < N; i++)
  {
    (*pvert_list_)[i]->SetRenderPos( Unify((*pvert_list_)[i]->Position()) );
    if ((*pvert_list_)[i]->isFixed())
    {
      fixed_vert_++;
    }
    if ((*pvert_list_)[i]->isBase())
    {
      base_vert_++;
    }
  }
}


point WireFrame::Unify(Vec3f p)
{
  return (p - center_pos_) * scaleV_;
}


void WireFrame::SimplifyFrame()
{
  int N = SizeOfVertList();
  int M = SizeOfEdgeList();

  vector<bool> delete_vert;
  delete_vert.resize(N);
  fill(delete_vert.begin(), delete_vert.end(), false);

  vector<bool> delete_edge;
  delete_edge.resize(M);
  fill(delete_edge.begin(), delete_edge.end(), false);

  for (int i = 0; i < N; i++)
  {
    WF_vert *u = (*pvert_list_)[i];
    if (u->Degree() == 2)
    {
      WF_edge *e1 = (*pvert_list_)[i]->pedge_;
      WF_edge *e2 = (*pvert_list_)[i]->pedge_->pnext_;
      WF_vert *v1 = e1->pvert_;
      WF_vert *v2 = e2->pvert_;

      if (ArcHeight(u->Position(), v1->Position(), v2->Position()) < delta_tol_)
      {
        delete_vert[u->ID()] = true;
        delete_edge[e1->ID()] = true;
        delete_edge[e2->ID()] = true;

        WF_edge *e1_pair = e1->ppair_;
        WF_edge *e2_pair = e2->ppair_;
        e1_pair->pvert_ = v2;
        e2_pair->pvert_ = v1;
        e1_pair->ppair_ = e2_pair;
        e2_pair->ppair_ = e1_pair;
      }
    }
  }

  vector<WF_vert*>::iterator itv = pvert_list_->begin();
  while (itv != pvert_list_->end())
  {
    if (delete_vert[(*itv)->ID()])
    {
      itv = pvert_list_->erase(itv);
    }
    else
    {
      itv++;
    }
  }

  vector<WF_edge*>::iterator ite = pedge_list_->begin();
  while (ite != pedge_list_->end())
  {
    if (delete_edge[(*ite)->ID()])
    {
      ite = pedge_list_->erase(ite);
    }
    else
    {
      ite++;
    }
  }

  Unify();
}


void WireFrame::ProjectBound(double len)
{
  if (base_vert_ == 0)
  {
    return;
  }

  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    WF_vert *u = (*pvert_list_)[i];
    if (u->isBase())
    {
      point v_pos = u->Position();
      v_pos.z() = minz_ - len;

      WF_vert *v = InsertVertex(v_pos);
      v->SetFixed(true);

      WF_edge *e = InsertEdge(u, v);
      e->SetPillar(true);
      e->ppair_->SetPillar(true);
    }
  }

  Unify();
}


void WireFrame::ModifyProjection(double len)
{
  if (SizeOfFixedVert() == 0)
  {
    return;
  }

  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    WF_vert *u = (*pvert_list_)[i];
    if (u->isFixed())
    {
      double x = u->Position().x();
      double y = u->Position().y();
      double z = u->Position().z();
      z = minz_ - len;
      u->SetPosition(x, y, z);
    }
  }
  Unify();
}


void WireFrame::MakeCeiling(vector<WF_edge*> &bound_e)
{
  if (bound_e.size() == 0)
  {
    return;
  }

  int M = SizeOfEdgeList();
  for (int i = 0; i < M; i++)
  {
    (*pedge_list_)[i]->SetCeiling(false);
  }

  int Mb = bound_e.size();
  for (int i = 0; i < Mb; i++)
  {
    WF_edge *e = bound_e[i];
    e->SetCeiling(true);
    e->ppair_->SetCeiling(true);
  }

  Unify();
}


void WireFrame::MakeBase(vector<WF_vert*> &base_v)
{
  if (base_v.size() == 0)
  {
    return;
  }

  int M = SizeOfEdgeList();
  for (int i = 0; i < M; i++)
  {
    (*pedge_list_)[i]->SetPillar(false);
  }

  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    (*pvert_list_)[i]->SetBase(false);
    (*pvert_list_)[i]->SetFixed(false);
  }

  int Nb = base_v.size();
  for (int i = 0; i < Nb; i++)
  {
    WF_vert *v = base_v[i];
    v->SetBase(true);
  }

  Unify();
}


void WireFrame::MakeSubGraph(vector<WF_edge*> &subg_e)
{
  if (subg_e.size() == 0)
  {
    return;
  }

  int N = SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    (*pvert_list_)[i]->SetSubgraph(false);
  }

  int M = SizeOfEdgeList();
  for (int i = 0; i < M; i++)
  {
    (*pedge_list_)[i]->SetSubgraph(false);
  }

  int Mb = subg_e.size();
  for (int i = 0; i < Mb; i++)
  {
    WF_edge *e = subg_e[i];
    e->SetSubgraph(true);
    e->ppair_->SetSubgraph(true);
    e->pvert_->SetSubgraph(true);
    e->ppair_->pvert_->SetSubgraph(true);
  }

  Unify();
}