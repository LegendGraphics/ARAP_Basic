#include "Arap.h"
using namespace std;
Arap::Arap()
{
	setupUi(this);

	m_SkullMesh = vtkSmartPointer< vtkOBJReader >::New();
	char *fileName = "D://Projects//arap//one-petal-sim.obj";
	m_SkullMesh->SetFileName(fileName);
	m_SkullMesh->Update();

	//vtkSmartPointer<vtkExtractEdges> extractEdges = vtkSmartPointer<vtkExtractEdges>::New();
	//extractEdges->SetInputConnection(m_SkullMesh->GetOutputPort());
	//extractEdges->Update();
	vtkSmartPointer<vtkPolyData> mesh = m_SkullMesh->GetOutput();
	vector<vector<int>> adj_list;
	vector<Eigen::Vector3i> face_list;

	for (vtkIdType i = 0; i != mesh->GetNumberOfPoints(); ++i) {
		vtkSmartPointer<vtkIdList> cellIdList = vtkSmartPointer<vtkIdList>::New();
		mesh->GetPointCells(i, cellIdList);
		vector<int> v_adj_list;
		for (vtkIdType j = 0; j != cellIdList->GetNumberOfIds(); ++j) {
			vtkSmartPointer<vtkIdList> pointIdList = vtkSmartPointer<vtkIdList>::New();
			mesh->GetCellPoints(cellIdList->GetId(j), pointIdList);
			if (pointIdList->GetId(0) == i) {
				v_adj_list.push_back(pointIdList->GetId(1));
				v_adj_list.push_back(pointIdList->GetId(2));
			}
			else if (pointIdList->GetId(1) == i){
				v_adj_list.push_back(pointIdList->GetId(0));
				v_adj_list.push_back(pointIdList->GetId(2));
			}
			else {
				v_adj_list.push_back(pointIdList->GetId(0));
				v_adj_list.push_back(pointIdList->GetId(1));
			}
		}
		sort(v_adj_list.begin(), v_adj_list.end());
		vector<int>::iterator iter = unique(v_adj_list.begin(), v_adj_list.end());
		v_adj_list.erase(iter, v_adj_list.end());
		v_adj_list.shrink_to_fit();
		adj_list.push_back(v_adj_list);
	}
	for (vtkIdType i = 0; i != mesh->GetNumberOfCells(); ++i) {
		vtkSmartPointer<vtkIdList> pointIdList = vtkSmartPointer<vtkIdList>::New();
		mesh->GetCellPoints(i, pointIdList);
		vector<int> f;
		f.push_back(pointIdList->GetId(0));
		f.push_back(pointIdList->GetId(1));
		f.push_back(pointIdList->GetId(2));
		sort(f.begin(), f.end());
		face_list.push_back(Eigen::Map<Eigen::Vector3i>(&f[0]));
	}

	int meshNum = m_SkullMesh->GetOutput()->GetNumberOfPoints();
	double *skullTemplate = new double[3*meshNum];
	for (int i = 0; i < meshNum; ++i) {
		double* temp = m_SkullMesh->GetOutput()->GetPoint(i);
		skullTemplate[3*i] = temp[0];
		skullTemplate[3*i+1] = temp[1];
		skullTemplate[3*i+2] = temp[2];
	}

	model = new Deform(skullTemplate, meshNum, adj_list, face_list);
	//MatrixXd Di = MatrixXd::Zero(2000, 2000);

	vtkSmartPointer< vtkPolyDataMapper > mapper = vtkSmartPointer< vtkPolyDataMapper >::New();
	mapper->SetInputConnection(m_SkullMesh->GetOutputPort());

	vtkSmartPointer< vtkActor > actor = vtkSmartPointer< vtkActor >::New();
	actor->SetMapper(mapper);

	vtkSmartPointer< vtkRenderer > render = vtkSmartPointer< vtkRenderer >::New();
	render->AddActor(actor);
	m_QvtkWidget->GetRenderWindow()->AddRenderer(render);
	m_QvtkWidget->GetRenderWindow()->Render();

	float *temp = model->get_P_Prime();
	vtkSmartPointer< vtkFloatArray > skullModelCoord = vtkSmartPointer< vtkFloatArray >::New();
	skullModelCoord->SetNumberOfComponents(3);
	skullModelCoord->SetNumberOfTuples(mesh->GetNumberOfPoints());
	skullModelCoord->SetArray(temp, mesh->GetNumberOfPoints()*3, 1);
	vtkSmartPointer< vtkPoints > skullModelPts = vtkSmartPointer< vtkPoints >::New();
	skullModelPts->SetData(skullModelCoord);
	vtkSmartPointer< vtkPolyData > skullModelPolydata = vtkSmartPointer< vtkPolyData >::New();
	skullModelPolydata->SetPoints(skullModelPts);
	vtkSmartPointer< vtkVertexGlyphFilter > skullModelVertexGlyphFilter = vtkSmartPointer< vtkVertexGlyphFilter >::New();
	skullModelVertexGlyphFilter->SetInput(skullModelPolydata);
	//vtkSmartPointer<vtkPolyDataConnectivityFilter> cfilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
	//cfilter->SetInput(skullModelVertexGlyphFilter->GetOutput());
	//cfilter->SetExtractionModeToLargestRegion();
	//cfilter->Update();
	vtkSmartPointer< vtkPolyDataMapper > skullModelmapper = vtkSmartPointer< vtkPolyDataMapper >::New();
	skullModelmapper->SetInput(skullModelVertexGlyphFilter->GetOutput());
	
	m_SkullModelActor = vtkSmartPointer< vtkActor >::New();
	m_SkullModelActor->SetMapper(skullModelmapper);
	m_SkullModelActor->GetProperty()->SetPointSize(1.5);
	//m_SkullModelActor->GetProperty()->SetRepresentationToPoints();
	m_SkullModelActor->GetProperty()->SetDiffuseColor(0.9, 0.447, 0.0745);
	render->AddActor(m_SkullModelActor);

	connect( m_PushButtonDoDeformIter, SIGNAL( clicked() ), this, SLOT( update_result() ) );
	connect( m_PushButtonDoDeform, SIGNAL( clicked() ), this, SLOT( update_final() ) );
}

Arap::~Arap()
{
}

void Arap::update_result()
{
	float *src = model->get_P_Prime();
	//for ()
	vector<double> T;
	vector<int> idx_T;
	for (int i = 0; i != m_SkullMesh->GetOutput()->GetNumberOfPoints(); ++i) {
		if (abs(src[3*i+2]-0) < 1){// || abs(src[3*i+2]-12) < 1) {
			T.push_back(src[3*i]);
			T.push_back(src[3*i+1]);
			T.push_back(src[3*i+2]);
			idx_T.push_back(i);
		}
		else if (abs(src[3*i+2]-173) < 1){// || abs(src[3*i+2]-162) < 1) {
			T.push_back(src[3*i]+50);
			T.push_back(src[3*i+1]+50);
			T.push_back(src[3*i+2]-50);
			idx_T.push_back(i);
		}
	}
	double delta;
	float *temp = model->do_Deform_Iter(delta);
	cout << "delta: " << delta << endl;
	vtkSmartPointer< vtkPolyData > skullModelPolydata = vtkSmartPointer< vtkPolyData >::New();
	skullModelPolydata->DeepCopy(m_SkullMesh->GetOutput());
	for (int i = 0; i != m_SkullMesh->GetOutput()->GetNumberOfPoints(); ++i) {
		vtkPoints *handle_deform = skullModelPolydata->GetPoints();
		//handle_deform->SetPoint(i, temp[3*i], temp[3*i+1], temp[3*i+2]);
		double *p = handle_deform->GetPoint(i);
		//cout << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
		handle_deform->SetPoint(i, temp[3*i], temp[3*i+1], temp[3*i+2]);
		p = handle_deform->GetPoint(i);
		//cout << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
	}

	vtkSmartPointer< vtkPolyDataMapper > skullModelmapper = vtkSmartPointer< vtkPolyDataMapper >::New();
	skullModelmapper->SetInput(skullModelPolydata);
	m_SkullModelActor->SetMapper(skullModelmapper);
	m_QvtkWidget->GetRenderWindow()->Render();
}

void Arap::update_final()
{
	float *src = model->get_P_Prime();
	//for ()
	vector<double> T;
	vector<int> idx_T;
	for (int i = 0; i != m_SkullMesh->GetOutput()->GetNumberOfPoints(); ++i) {
		if (abs(src[3 * i + 2] - 900) < 5){// || abs(src[3*i+2]-12) < 1) {
			T.push_back(src[3 * i] + 50);
			T.push_back(src[3 * i + 1] + 50);
			T.push_back(src[3 * i + 2]);
			idx_T.push_back(i);
		}
		else {//if (abs(src[3*i+2]-46) < 1){// || abs(src[3*i+2]-162) < 1) {
			T.push_back(src[3 * i] - 50);
			T.push_back(src[3 * i + 1] - 50);
			T.push_back(src[3 * i + 2]);
			idx_T.push_back(i);
		}
	}

	//T.push_back(0);
	//T.push_back(-18);
	//T.push_back(0);
	//T.push_back(50);
	//T.push_back(40);
	//T.push_back(150);
	//idx_T.push_back(0);
	//idx_T.push_back(25);

	double delta;
	int max_iter = 500;
	int iter = 0;
	model->set_linear_sys(T, idx_T);
	do {
		float *temp = model->do_Deform_Iter(delta);
		cout << "delta: " << delta << endl;
		++iter;
		vtkSmartPointer< vtkPolyData > skullModelPolydata = vtkSmartPointer< vtkPolyData >::New();
		skullModelPolydata->DeepCopy(m_SkullMesh->GetOutput());
		for (int i = 0; i != m_SkullMesh->GetOutput()->GetNumberOfPoints(); ++i) {
			vtkPoints *handle_deform = skullModelPolydata->GetPoints();
			//handle_deform->SetPoint(i, temp[3*i], temp[3*i+1], temp[3*i+2]);
			double *p = handle_deform->GetPoint(i);
			//cout << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
			handle_deform->SetPoint(i, temp[3*i], temp[3*i+1], temp[3*i+2]);
			p = handle_deform->GetPoint(i);
			//cout << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
		}
		//system("pause");

		vtkSmartPointer< vtkPolyDataMapper > skullModelmapper = vtkSmartPointer< vtkPolyDataMapper >::New();
		skullModelmapper->SetInput(skullModelPolydata);
		m_SkullModelActor->SetMapper(skullModelmapper);
		m_QvtkWidget->GetRenderWindow()->Render();
	}while(delta > 1e-2 && iter < max_iter);
}