#pragma once

#include <warn/push>
#include <warn/ignore/all>
#include <vtkDataArrayAccessor.h>
#include <vtkAssume.h>
#include <warn/pop>

namespace inviwo{
    namespace util{
        template <unsigned N>
struct VTKToVector {
    using TensorFieldType =
        std::conditional_t<N == 2, TensorField2D, std::conditional_t<N == 3, TensorField3D, void>>;
    using matN = typename TensorFieldType::matN;

    std::shared_ptr<std::vector<matN>> vec;

    VTKToVector() : vec(std::make_shared<std::vector<matN>>()) {}

    template <typename TensorArray>
    void operator()(TensorArray* tensors) {
        vec->clear();
        // This allows the compiler to optimize for the AOS array stride.
        const auto numComponents = tensors->GetNumberOfComponents();
        VTK_ASSUME(numComponents == N * N);

        vtkDataArrayAccessor<TensorArray> t(tensors);

        const vtkIdType numTensors = tensors->GetNumberOfTuples();

        vec->reserve(numTensors * numComponents);

        for (vtkIdType tupleIdx = 0; tupleIdx < numTensors; ++tupleIdx) {
            // Get compiles to inlined optimizable raw memory accesses for
            // vtkGenericDataArray subclasses.
            if constexpr (N == 2) {
                vec->emplace_back(matN(t.Get(tupleIdx, 0), t.Get(tupleIdx, 1), t.Get(tupleIdx, 2),
                                       t.Get(tupleIdx, 3)));
            }
            if constexpr (N == 3) {
                vec->emplace_back(matN(t.Get(tupleIdx, 0), t.Get(tupleIdx, 1), t.Get(tupleIdx, 2),
                                       t.Get(tupleIdx, 3), t.Get(tupleIdx, 4), t.Get(tupleIdx, 5),
                                       t.Get(tupleIdx, 6), t.Get(tupleIdx, 7), t.Get(tupleIdx, 8)));
            }
        }
    }
};
    }
}