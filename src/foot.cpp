
#include "foot.h"
#include <gsim/gs_ogl.h>
#include "draw_primitives.h"
#include "app_main.h"

FootGoal::FootGoal(GsVec p, float a)
{
pos=p;
angle=a;
leftFoot = new Foot(GsVec(0,0,0));
rightFoot = new Foot(GsVec(0,0,0));
leftFoot->model->model(App->walk->leftfootMod);
rightFoot->model->model(App->walk->rightfootMod);
//leftFoot->Rotate(a);
//rightFoot->Rotate(a);

modg = new SnGroup();
g = new SnGroup();
tfm = new SnTransform();
g->add(tfm);
modg->add(leftFoot->g);
modg->add(rightFoot->g);
//modg->separator(true);
g->add(modg);
g->separator(true);

tfm->get().roty(gs_torad(a));
tfm->get().setrans(p);

}
void FootGoal::Rotate(float deg)
{
angle = deg;
tfm->get().roty(gs_torad(deg));
}
void FootGoal::Translate(GsVec p)
{
pos = p;
tfm->get().setrans(p);
}
Foot::Foot(GsVec ct)
{
adjusted = false;
angle = 0;
offset.set(0,0,0);
tfm = new SnTransform();
Translate(ct);
g = new SnGroup();
model = new SnModel();
g->add(tfm);
g->add(model);
g->separator(true);

}

void Foot::Rotate(float deg)
{
angle = deg;
tfm->get().roty(gs_torad(deg));
}
void Foot::Translate(GsVec pos)
{
tfm->get().setrans(pos);
}

void Foot::draw()
{
glLineWidth(0.2f);
drawLine(fl,fr);
drawLine(fr,br);
drawLine(br,bl);
drawLine(bl,fl);

fl.set(-0.04f,0.08f,0);
fr.set(0.04f,0.08f,0);
bl.set(-0.04f,-0.08f,0);
br.set(0.04f,-0.08f,0);
fl+=offset;
fr+=offset;
bl+=offset;
br+=offset;

fl.rotz(gs_torad(angle));
fr.rotz(gs_torad(angle));
br.rotz(gs_torad(angle));
bl.rotz(gs_torad(angle));

fl+=c;
fr+=c;
bl+=c;
br+=c;


}
