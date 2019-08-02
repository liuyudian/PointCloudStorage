#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PointCloudManage.h"
#include "vtkAutoInit.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);

VTK_MODULE_INIT(vtkInteractionStyle);
class PointCloudManage : public QMainWindow
{
	Q_OBJECT

public:
	PointCloudManage(QWidget *parent = Q_NULLPTR);

private:
	Ui::PointCloudManageClass ui;
};
