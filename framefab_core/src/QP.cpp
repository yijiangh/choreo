#include "QP.h"

std::ostream& operator<<(std::ostream& out, const QP& qp)
{
	out << qp.report();
	return out;
}