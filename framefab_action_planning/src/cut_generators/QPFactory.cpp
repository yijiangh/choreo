#include <cut_generators/QPFactory.h>

QP *QPFactory::make(QPFactory::QPType t, bool _storeVariables)
{
	QP* qp = NULL;

	switch (t){

	case QPTYPE_BEGIN:
		return qp;

	case QPMOSEKT:
		qp = new QPMosek();
		break;

	case QPTYPE_END:
		return qp;
	}

	qp->setStoreVariables(_storeVariables);
	return qp;
}

void QPFactory::typeToString(QPFactory::QPType t, std::string &s)
{
	switch (t){
	case QPTYPE_BEGIN:	s = std::string("Invalid QP Type");
		break;
	case QPTYPE_END:	s = std::string("Invalid QP Type");
		break;
		break;
	case QPMOSEKT:		s = std::string("QPMosek");
		break;
	}
}
