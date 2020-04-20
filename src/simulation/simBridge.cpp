
# include "SharedMemory.h"
# include "SimulatorMessage.h"

//# include "../../3rdparty/pybind11/pybind11"

# include "pybind11/pybind11.h"

int add(int i, int j)
{
    return i + j;
}

namespace py = pybind11;


PYBIND11_MODULE(example, m)
{
    m.doc() = "pybind11 example plugin"; // 可选的模块说明

    m.def("add", &add, "A function which adds two numbers");
}



