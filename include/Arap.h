#ifndef _Arap_H
#define _Arap_H

#include <QDockWidget>
#include "ui_Arap.h"

#include "vtkSmartPointer.h"
#include "vtkOBJReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkExtractEdges.h"
#include "vtkIdList.h"
#include "vtkMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkFloatArray.h"
#include "vtkVertexGlyphFilter.h"
#include "vtkProperty.h"

#include "Deform.h"

#include <iterator>
#include <algorithm>

class Arap : public QDockWidget, public Ui::Arap
{
	Q_OBJECT
	
public:
	Arap();
	~Arap();

private slots:
	void update_result();
	void update_final();

private:
	vtkSmartPointer< vtkOBJReader > m_SkullMesh;
	Deform *model;
	vtkSmartPointer< vtkActor > m_SkullModelActor;
};

#endif 