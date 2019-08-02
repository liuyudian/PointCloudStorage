#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PointCloudMange.h"
#include "vtkAutoInit.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);

VTK_MODULE_INIT(vtkInteractionStyle);
class PointCloudMange : public QMainWindow
{
	Q_OBJECT

public:
	PointCloudMange(QWidget *parent = Q_NULLPTR);

private:
	Ui::PointCloudMangeClass ui;
};
