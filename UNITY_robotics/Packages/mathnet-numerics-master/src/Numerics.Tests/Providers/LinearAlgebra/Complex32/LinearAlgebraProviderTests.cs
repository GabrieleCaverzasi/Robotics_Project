// <copyright file="LinearAlgebraProviderTests.cs" company="Math.NET">
// Math.NET Numerics, part of the Math.NET Project
// http://numerics.mathdotnet.com
// http://github.com/mathnet/mathnet-numerics
//
// Copyright (c) 2009-2016 Math.NET
//
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// </copyright>

using System;
using System.Collections.Generic;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex32;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.Providers.LinearAlgebra;
using NUnit.Framework;

namespace MathNet.Numerics.Tests.Providers.LinearAlgebra.Complex32
{
    using Numerics;

    /// <summary>
    /// Base class for linear algebra provider tests.
    /// </summary>
    [TestFixture, Category("LAProvider")]
    public class LinearAlgebraProviderTests
    {
        /// <summary>
        /// The Y Complex32 test vector.
        /// </summary>
        readonly Complex32[] _y = {new Complex32(1.1f, 0f), 2.2f, 3.3f, 4.4f, 5.5f};

        /// <summary>
        /// The X Complex32 test vector.
        /// </summary>
        readonly Complex32[] _x = {new Complex32(6.6f, 0f), 7.7f, 8.8f, 9.9f, 10.1f};

        static readonly IContinuousDistribution Dist = new Normal();

        /// <summary>
        /// Test matrix to use.
        /// </summary>
        readonly IDictionary<string, DenseMatrix> _matrices = new Dictionary<string, DenseMatrix>
            {
                {"Singular3x3", DenseMatrix.OfArray(new[,] {{new Complex32(1.0f, 0.0f), 1.0f, 2.0f}, {1.0f, 1.0f, 2.0f}, {1.0f, 1.0f, 2.0f}})},
                {"Square3x3", DenseMatrix.OfArray(new[,] {{new Complex32(-1.1f, 0.0f), -2.2f, -3.3f}, {0.0f, 1.1f, 2.2f}, {-4.4f, 5.5f, 6.6f}})},
                {"Square4x4", DenseMatrix.OfArray(new[,] {{new Complex32(-1.1f, 0.0f), -2.2f, -3.3f, -4.4f}, {0.0f, 1.1f, 2.2f, 3.3f}, {1.0f, 2.1f, 6.2f, 4.3f}, {-4.4f, 5.5f, 6.6f, -7.7f}})},
                {"Singular4x4", DenseMatrix.OfArray(new[,] {{new Complex32(-1.1f, 0.0f), -2.2f, -3.3f, -4.4f}, {-1.1f, -2.2f, -3.3f, -4.4f}, {-1.1f, -2.2f, -3.3f, -4.4f}, {-1.1f, -2.2f, -3.3f, -4.4f}})},
                {"Tall3x2", DenseMatrix.OfArray(new[,] {{new Complex32(-1.1f, 0.0f), -2.2f}, {0.0f, 1.1f}, {-4.4f, 5.5f}})},
                {"Wide2x3", DenseMatrix.OfArray(new[,] {{new Complex32(-1.1f, 0.0f), -2.2f, -3.3f}, {0.0f, 1.1f, 2.2f}})},
                {"Tall50000x10", DenseMatrix.CreateRandom(50000, 10, Dist)},
                {"Wide10x50000", DenseMatrix.CreateRandom(10, 50000, Dist)},
                {"Square1000x1000", DenseMatrix.CreateRandom(1000, 1000, Dist)}
            };

        /// <summary>
        /// Can add a vector to scaled vector
        /// </summary>
        [Test]
        public void CanAddVectorToScaledVectorComplex32()
        {
            var result = new Complex32[_y.Length];

            LinearAlgebraControl.Provider.AddVectorToScaledVector(_y, 0, _x, result);
            for (var i = 0; i < _y.Length; i++)
            {
                Assert.AreEqual(_y[i], result[i]);
            }

            Array.Copy(_y, result, _y.Length);
            LinearAlgebraControl.Provider.AddVectorToScaledVector(result, 1, _x, result);
            for (var i = 0; i < _y.Length; i++)
            {
                Assert.AreEqual(_y[i] + _x[i], result[i]);
            }

            Array.Copy(_y, result, _y.Length);
            LinearAlgebraControl.Provider.AddVectorToScaledVector(result, (Complex32) Math.PI, _x, result);
            for (var i = 0; i < _y.Length; i++)
            {
                AssertHelpers.AlmostEqualRelative(_y[i] + ((Complex32) Math.PI*_x[i]), result[i], 5);
            }
        }

        /// <summary>
        /// Can scale an array.
        /// </summary>
        [Test]
        public void CanScaleArray()
        {
            var result = new Complex32[_y.Length];

            LinearAlgebraControl.Provider.ScaleArray(1, _y, result);
            for (var i = 0; i < _y.Length; i++)
            {
                Assert.AreEqual(_y[i], result[i]);
            }

            Array.Copy(_y, result, _y.Length);
            LinearAlgebraControl.Provider.ScaleArray((Complex32) Math.PI, result, result);
            for (var i = 0; i < _y.Length; i++)
            {
                AssertHelpers.AlmostEqualRelative(_y[i]*(Complex32) Math.PI, result[i], 5);
            }
        }

        /// <summary>
        /// Can compute the dot product.
        /// </summary>
        [Test]
        public void CanComputeDotProduct()
        {
            var result = LinearAlgebraControl.Provider.DotProduct(_x, _y);
            AssertHelpers.AlmostEqualRelative(152.35f, result, 5);
        }

        /// <summary>
        /// Can add two arrays.
        /// </summary>
        [Test]
        public void CanAddArrays()
        {
            var result = new Complex32[_y.Length];
            LinearAlgebraControl.Provider.AddArrays(_x, _y, result);
            for (var i = 0; i < result.Length; i++)
            {
                Assert.AreEqual(_x[i] + _y[i], result[i]);
            }
        }

        /// <summary>
        /// Can subtract two arrays.
        /// </summary>
        [Test]
        public void CanSubtractArrays()
        {
            var result = new Complex32[_y.Length];
            LinearAlgebraControl.Provider.SubtractArrays(_x, _y, result);
            for (var i = 0; i < result.Length; i++)
            {
                Assert.AreEqual(_x[i] - _y[i], result[i]);
            }
        }

        /// <summary>
        /// Can pointwise multiply two arrays.
        /// </summary>
        [Test]
        public void CanPointWiseMultiplyArrays()
        {
            var result = new Complex32[_y.Length];
            LinearAlgebraControl.Provider.PointWiseMultiplyArrays(_x, _y, result);
            for (var i = 0; i < result.Length; i++)
            {
                Assert.AreEqual(_x[i]*_y[i], result[i]);
            }
        }

        /// <summary>
        /// Can pointwise divide two arrays.
        /// </summary>
        [Test]
        public void CanPointWiseDivideArrays()
        {
            var result = new Complex32[_y.Length];
            LinearAlgebraControl.Provider.PointWiseDivideArrays(_x, _y, result);
            for (var i = 0; i < result.Length; i++)
            {
                Assert.AreEqual(_x[i]/_y[i], result[i]);
            }
        }

        /// <summary>
        /// Can compute L1 norm.
        /// </summary>
        [Test]
        public void CanComputeMatrixL1Norm()
        {
            var matrix = _matrices["Square3x3"];
            var norm = LinearAlgebraControl.Provider.MatrixNorm(Norm.OneNorm, matrix.RowCount, matrix.ColumnCount, matrix.Values);
            AssertHelpers.AlmostEqualRelative(12.1f, norm, 5);
        }

        /// <summary>
        /// Can compute Frobenius norm.
        /// </summary>
        [Test]
        public void CanComputeMatrixFrobeniusNorm()
        {
            var matrix = _matrices["Square3x3"];
            var norm = LinearAlgebraControl.Provider.MatrixNorm(Norm.FrobeniusNorm, matrix.RowCount, matrix.ColumnCount, matrix.Values);
            AssertHelpers.AlmostEqualRelative(10.777754868246f, norm, 5);
        }

        /// <summary>
        /// Can compute Infinity norm.
        /// </summary>
        [Test]
        public void CanComputeMatrixInfinityNorm()
        {
            var matrix = _matrices["Square3x3"];
            var norm = LinearAlgebraControl.Provider.MatrixNorm(Norm.InfinityNorm, matrix.RowCount, matrix.ColumnCount, matrix.Values);
            Assert.AreEqual(16.5, norm);
        }

        /// <summary>
        /// Can multiply two square matrices.
        /// </summary>
        [Test]
        public void CanMultiplySquareMatrices()
        {
            var x = _matrices["Singular3x3"];
            var y = _matrices["Square3x3"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiply(x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    AssertHelpers.AlmostEqualRelative(x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can multiply a wide and tall matrix.
        /// </summary>
        [Test]
        public void CanMultiplyWideAndTallMatrices()
        {
            var x = _matrices["Wide2x3"];
            var y = _matrices["Tall3x2"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiply(x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    AssertHelpers.AlmostEqualRelative(x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can multiply a tall and wide matrix.
        /// </summary>
        [Test]
        public void CanMultiplyTallAndWideMatrices()
        {
            var x = _matrices["Tall3x2"];
            var y = _matrices["Wide2x3"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiply(x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    AssertHelpers.AlmostEqualRelative(x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can multiply two square matrices.
        /// </summary>
        [Test]
        public void CanMultiplySquareMatricesWithUpdate()
        {
            var x = _matrices["Singular3x3"];
            var y = _matrices["Square3x3"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiplyWithUpdate(Transpose.DontTranspose, Transpose.DontTranspose, 2.2f, x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, 1.0f, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    AssertHelpers.AlmostEqualRelative(2.2f*x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can multiply a wide and tall matrix.
        /// </summary>
        [Test]
        public void CanMultiplyWideAndTallMatricesWithUpdate()
        {
            var x = _matrices["Wide2x3"];
            var y = _matrices["Tall3x2"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiplyWithUpdate(Transpose.DontTranspose, Transpose.DontTranspose, 2.2f, x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, 1.0f, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    AssertHelpers.AlmostEqualRelative(2.2f*x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can multiply a tall and wide matrix.
        /// </summary>
        [Test]
        public void CanMultiplyTallAndWideMatricesWithUpdate()
        {
            var x = _matrices["Tall3x2"];
            var y = _matrices["Wide2x3"];
            var c = new DenseMatrix(x.RowCount, y.ColumnCount);

            LinearAlgebraControl.Provider.MatrixMultiplyWithUpdate(Transpose.DontTranspose, Transpose.DontTranspose, 2.2f, x.Values, x.RowCount, x.ColumnCount, y.Values, y.RowCount, y.ColumnCount, 1.0f, c.Values);

            for (var i = 0; i < c.RowCount; i++)
            {
                for (var j = 0; j < c.ColumnCount; j++)
                {
                    var test = 2.2f*x.Row(i)*y.Column(j);

                    // if they are both close to zero, skip
                    if (Math.Abs(test.Real) < 1e-7 && Math.Abs(c[i, j].Real) < 1e-7)
                    {
                        continue;
                    }

                    AssertHelpers.AlmostEqualRelative(2.2f*x.Row(i)*y.Column(j), c[i, j], 5);
                }
            }
        }

        /// <summary>
        /// Can compute the LU factor of a matrix.
        /// </summary>
        [Test]
        public void CanComputeLuFactor()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            var ipiv = new int[matrix.RowCount];

            LinearAlgebraControl.Provider.LUFactor(a, matrix.RowCount, ipiv);

            AssertHelpers.AlmostEqualRelative(a[0], -4.4f, 5);
            AssertHelpers.AlmostEqualRelative(a[1], 0.25f, 5);
            AssertHelpers.AlmostEqualRelative(a[2], 0f, 5);
            AssertHelpers.AlmostEqualRelative(a[3], 5.5f, 5);
            AssertHelpers.AlmostEqualRelative(a[4], -3.575f, 5);
            AssertHelpers.AlmostEqualRelative(a[5], -0.307692307692308f, 5);
            AssertHelpers.AlmostEqualRelative(a[6], 6.6f, 5);
            AssertHelpers.AlmostEqualRelative(a[7], -4.95f, 5);
            AssertHelpers.AlmostEqualRelative(a[8], 0.676923076923077f, 5);
            Assert.AreEqual(ipiv[0], 2);
            Assert.AreEqual(ipiv[1], 2);
            Assert.AreEqual(ipiv[2], 2);
        }

        /// <summary>
        /// Can compute the inverse of a matrix using LU factorization.
        /// </summary>
        [Test]
        public void CanComputeLuInverse()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            LinearAlgebraControl.Provider.LUInverse(a, matrix.RowCount);

            AssertHelpers.AlmostEqualRelative(a[0], -0.454545454545454f, 5);
            AssertHelpers.AlmostEqualRelative(a[1], -0.909090909090908f, 5);
            AssertHelpers.AlmostEqualRelative(a[2], 0.454545454545454f, 5);
            AssertHelpers.AlmostEqualRelative(a[3], -0.340909090909090f, 5);
            AssertHelpers.AlmostEqualRelative(a[4], -2.045454545454543f, 5);
            AssertHelpers.AlmostEqualRelative(a[5], 1.477272727272726f, 5);
            AssertHelpers.AlmostEqualRelative(a[6], -0.113636363636364f, 5);
            AssertHelpers.AlmostEqualRelative(a[7], 0.227272727272727f, 5);
            AssertHelpers.AlmostEqualRelative(a[8], -0.113636363636364f, 5);
        }

        /// <summary>
        /// Can compute the inverse of a matrix using LU factorization
        /// using a previously factored matrix.
        /// </summary>
        [Test]
        public void CanComputeLuInverseOnFactoredMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            var ipiv = new int[matrix.RowCount];

            LinearAlgebraControl.Provider.LUFactor(a, matrix.RowCount, ipiv);
            LinearAlgebraControl.Provider.LUInverseFactored(a, matrix.RowCount, ipiv);

            AssertHelpers.AlmostEqualRelative(a[0], -0.454545454545454f, 5);
            AssertHelpers.AlmostEqualRelative(a[1], -0.909090909090908f, 5);
            AssertHelpers.AlmostEqualRelative(a[2], 0.454545454545454f, 5);
            AssertHelpers.AlmostEqualRelative(a[3], -0.340909090909090f, 5);
            AssertHelpers.AlmostEqualRelative(a[4], -2.045454545454543f, 5);
            AssertHelpers.AlmostEqualRelative(a[5], 1.477272727272726f, 5);
            AssertHelpers.AlmostEqualRelative(a[6], -0.113636363636364f, 5);
            AssertHelpers.AlmostEqualRelative(a[7], 0.227272727272727f, 5);
            AssertHelpers.AlmostEqualRelative(a[8], -0.113636363636364f, 5);
        }

        /// <summary>
        /// Can solve Ax=b using LU factorization.
        /// </summary>
        [Test]
        public void CanSolveUsingLU()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            LinearAlgebraControl.Provider.LUSolve(2, a, matrix.RowCount, b);

            AssertHelpers.AlmostEqualRelative(b[0], -1.477272727272726f, 5);
            AssertHelpers.AlmostEqualRelative(b[1], -4.318181818181815f, 5);
            AssertHelpers.AlmostEqualRelative(b[2], 3.068181818181816f, 5);
            AssertHelpers.AlmostEqualRelative(b[3], -4.204545454545451f, 5);
            AssertHelpers.AlmostEqualRelative(b[4], -12.499999999999989f, 5);
            AssertHelpers.AlmostEqualRelative(b[5], 8.522727272727266f, 5);

            NotModified(matrix.RowCount, matrix.ColumnCount, a, matrix);
        }

        /// <summary>
        /// Can solve Ax=b using LU factorization using a factored matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingLUOnFactoredMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            var ipiv = new int[matrix.RowCount];
            LinearAlgebraControl.Provider.LUFactor(a, matrix.RowCount, ipiv);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            LinearAlgebraControl.Provider.LUSolveFactored(2, a, matrix.RowCount, ipiv, b);

            AssertHelpers.AlmostEqualRelative(b[0], -1.477272727272726f, 5);
            AssertHelpers.AlmostEqualRelative(b[1], -4.318181818181815f, 5);
            AssertHelpers.AlmostEqualRelative(b[2], 3.068181818181816f, 5);
            AssertHelpers.AlmostEqualRelative(b[3], -4.204545454545451f, 5);
            AssertHelpers.AlmostEqualRelative(b[4], -12.499999999999989f, 5);
            AssertHelpers.AlmostEqualRelative(b[5], 8.522727272727266f, 5);
        }

        /// <summary>
        /// Can compute the <c>Cholesky</c> factorization.
        /// </summary>
        [Test]
        public void CanComputeCholeskyFactor()
        {
            var matrix = new Complex32[] {1, 1, 1, 1, 1, 5, 5, 5, 1, 5, 14, 14, 1, 5, 14, 15};
            LinearAlgebraControl.Provider.CholeskyFactor(matrix, 4);
            Assert.AreEqual(matrix[0].Real, 1);
            Assert.AreEqual(matrix[1].Real, 1);
            Assert.AreEqual(matrix[2].Real, 1);
            Assert.AreEqual(matrix[3].Real, 1);
            Assert.AreEqual(matrix[4].Real, 0);
            Assert.AreEqual(matrix[5].Real, 2);
            Assert.AreEqual(matrix[6].Real, 2);
            Assert.AreEqual(matrix[7].Real, 2);
            Assert.AreEqual(matrix[8].Real, 0);
            Assert.AreEqual(matrix[9].Real, 0);
            Assert.AreEqual(matrix[10].Real, 3);
            Assert.AreEqual(matrix[11].Real, 3);
            Assert.AreEqual(matrix[12].Real, 0);
            Assert.AreEqual(matrix[13].Real, 0);
            Assert.AreEqual(matrix[14].Real, 0);
            Assert.AreEqual(matrix[15].Real, 1);
        }

        /// <summary>
        /// Can solve Ax=b using Cholesky factorization.
        /// </summary>
        [Test]
        public void CanSolveUsingCholesky()
        {
            var matrix = new DenseMatrix(3, 3, new Complex32[] {1, 1, 1, 1, 2, 3, 1, 3, 6});
            var a = new Complex32[] {1, 1, 1, 1, 2, 3, 1, 3, 6};

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            LinearAlgebraControl.Provider.CholeskySolve(a, 3, b, 2);

            AssertHelpers.AlmostEqualRelative(b[0], 0, 5);
            AssertHelpers.AlmostEqualRelative(b[1], 1, 5);
            AssertHelpers.AlmostEqualRelative(b[2], 0, 5);
            AssertHelpers.AlmostEqualRelative(b[3], 3, 5);
            AssertHelpers.AlmostEqualRelative(b[4], 1, 5);
            AssertHelpers.AlmostEqualRelative(b[5], 0, 5);

            NotModified(3, 3, a, matrix);
        }

        /// <summary>
        /// Can solve Ax=b using LU factorization using a factored matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingCholeskyOnFactoredMatrix()
        {
            var a = new Complex32[] {1, 1, 1, 1, 2, 3, 1, 3, 6};

            LinearAlgebraControl.Provider.CholeskyFactor(a, 3);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            LinearAlgebraControl.Provider.CholeskySolveFactored(a, 3, b, 2);

            AssertHelpers.AlmostEqualRelative(b[0], 0, 5);
            AssertHelpers.AlmostEqualRelative(b[1], 1, 5);
            AssertHelpers.AlmostEqualRelative(b[2], 0, 5);
            AssertHelpers.AlmostEqualRelative(b[3], 3, 5);
            AssertHelpers.AlmostEqualRelative(b[4], 1, 5);
            AssertHelpers.AlmostEqualRelative(b[5], 0, 5);
        }

        /// <summary>
        /// Can compute QR factorization of a square matrix.
        /// </summary>
        [Test]
        public void CanComputeQRFactorSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var r = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, r, r.Length);

            var tau = new Complex32[3];
            var q = new Complex32[matrix.RowCount*matrix.RowCount];
            LinearAlgebraControl.Provider.QRFactor(r, matrix.RowCount, matrix.ColumnCount, q, tau);

            var mq = new DenseMatrix(matrix.RowCount, matrix.RowCount, q);
            var mr = new DenseMatrix(matrix.RowCount, matrix.ColumnCount, r).UpperTriangle();
            var a = mq*mr;

            for (var row = 0; row < matrix.RowCount; row++)
            {
                for (var col = 0; col < matrix.ColumnCount; col++)
                {
                    AssertHelpers.AlmostEqualRelative(matrix[row, col], a[row, col], 5);
                }
            }
        }

        /// <summary>
        /// Can compute QR factorization of a tall matrix.
        /// </summary>
        [Test]
        public void CanComputeQRFactorTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var r = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, r, r.Length);

            var tau = new Complex32[3];
            var q = new Complex32[matrix.RowCount*matrix.RowCount];
            LinearAlgebraControl.Provider.QRFactor(r, matrix.RowCount, matrix.ColumnCount, q, tau);

            var mr = new DenseMatrix(matrix.RowCount, matrix.ColumnCount, r).UpperTriangle();
            var mq = new DenseMatrix(matrix.RowCount, matrix.RowCount, q);
            var a = mq*mr;

            for (var row = 0; row < matrix.RowCount; row++)
            {
                for (var col = 0; col < matrix.ColumnCount; col++)
                {
                    AssertHelpers.AlmostEqualRelative(matrix[row, col], a[row, col], 5);
                }
            }
        }

        /// <summary>
        /// Can compute QR factorization of a wide matrix.
        /// </summary>
        [Test]
        public void CanComputeQRFactorWideMatrix()
        {
            var matrix = _matrices["Wide2x3"];
            var r = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, r, r.Length);

            var tau = new Complex32[3];
            var q = new Complex32[matrix.RowCount*matrix.RowCount];
            LinearAlgebraControl.Provider.QRFactor(r, matrix.RowCount, matrix.ColumnCount, q, tau);

            var mr = new DenseMatrix(matrix.RowCount, matrix.ColumnCount, r).UpperTriangle();
            var mq = new DenseMatrix(matrix.RowCount, matrix.RowCount, q);
            var a = mq*mr;

            for (var row = 0; row < matrix.RowCount; row++)
            {
                for (var col = 0; col < matrix.ColumnCount; col++)
                {
                    AssertHelpers.AlmostEqualRelative(matrix[row, col], a[row, col], 5);
                }
            }
        }

        /// <summary>
        /// Can compute thin QR factorization of a square matrix.
        /// </summary>
        [Test]
        public void CanComputeThinQRFactorSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var r = new Complex32[matrix.ColumnCount*matrix.ColumnCount];
            var tau = new Complex32[3];
            var q = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, q, q.Length);

            LinearAlgebraControl.Provider.ThinQRFactor(q, matrix.RowCount, matrix.ColumnCount, r, tau);

            var mq = new DenseMatrix(matrix.RowCount, matrix.ColumnCount, q);
            var mr = new DenseMatrix(matrix.ColumnCount, matrix.ColumnCount, r);
            var a = mq*mr;

            for (var row = 0; row < matrix.RowCount; row++)
            {
                for (var col = 0; col < matrix.ColumnCount; col++)
                {
                    AssertHelpers.AlmostEqualRelative(matrix[row, col], a[row, col], 5);
                }
            }
        }

        /// <summary>
        /// Can compute thin QR factorization of a tall matrix.
        /// </summary>
        [Test]
        public void CanComputeThinQRFactorTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var r = new Complex32[matrix.ColumnCount*matrix.ColumnCount];
            var tau = new Complex32[3];
            var q = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, q, q.Length);

            LinearAlgebraControl.Provider.ThinQRFactor(q, matrix.RowCount, matrix.ColumnCount, r, tau);

            var mq = new DenseMatrix(matrix.RowCount, matrix.ColumnCount, q);
            var mr = new DenseMatrix(matrix.ColumnCount, matrix.ColumnCount, r);
            var a = mq*mr;

            for (var row = 0; row < matrix.RowCount; row++)
            {
                for (var col = 0; col < matrix.ColumnCount; col++)
                {
                    AssertHelpers.AlmostEqualRelative(matrix[row, col], a[row, col], 5);
                }
            }
        }

        /// <summary>
        /// Can solve Ax=b using QR factorization with a square A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingQRSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x);

            NotModified(3, 3, a, matrix);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqualRelative(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqualRelative(mb[2, 0], b[2], 5);
            AssertHelpers.AlmostEqualRelative(mb[0, 1], b[3], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqualRelative(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using QR factorization with a tall A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingQRTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x);

            NotModified(3, 2, a, matrix);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqualRelative(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqualRelative(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 1], x[3], 5);
        }

        /// <summary>
        /// Can solve Ax=b using QR factorization with a square A matrix
        /// using a factored A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingQRSquareMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.RowCount];
            Array.Copy(matrix.Values, a, a.Length);

            var tau = new Complex32[matrix.ColumnCount];
            var q = new Complex32[matrix.ColumnCount*matrix.ColumnCount];
            LinearAlgebraControl.Provider.QRFactor(a, matrix.RowCount, matrix.ColumnCount, q, tau);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolveFactored(q, a, matrix.RowCount, matrix.ColumnCount, tau, b, 2, x);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqualRelative(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqualRelative(mb[2, 0], b[2], 5);
            AssertHelpers.AlmostEqualRelative(mb[0, 1], b[3], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqualRelative(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using QR factorization with a tall A matrix
        /// using a factored A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingQRTallMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var tau = new Complex32[matrix.ColumnCount];
            var q = new Complex32[matrix.RowCount*matrix.RowCount];
            LinearAlgebraControl.Provider.QRFactor(a, matrix.RowCount, matrix.ColumnCount, q, tau);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolveFactored(q, a, matrix.RowCount, matrix.ColumnCount, tau, b, 2, x);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqualRelative(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqualRelative(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 1], x[3], 5);
        }

        /// <summary>
        /// Can solve Ax=b using thin QR factorization with a square A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingThinQRSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x, QRMethod.Thin);

            NotModified(3, 3, a, matrix);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqualRelative(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqualRelative(mb[2, 0], b[2], 5);
            AssertHelpers.AlmostEqualRelative(mb[0, 1], b[3], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqualRelative(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using thin QR factorization with a tall A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingThinQRTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x, QRMethod.Thin);

            NotModified(3, 2, a, matrix);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqualRelative(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqualRelative(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 1], x[3], 5);
        }

        /// <summary>
        /// Can solve Ax=b using thin QR factorization with a square A matrix
        /// using a factored A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingThinQRSquareMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var tau = new Complex32[matrix.ColumnCount];
            var r = new Complex32[matrix.ColumnCount*matrix.ColumnCount];
            LinearAlgebraControl.Provider.ThinQRFactor(a, matrix.RowCount, matrix.ColumnCount, r, tau);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolveFactored(a, r, matrix.RowCount, matrix.ColumnCount, tau, b, 2, x, QRMethod.Thin);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqualRelative(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqualRelative(mb[2, 0], b[2], 5);
            AssertHelpers.AlmostEqualRelative(mb[0, 1], b[3], 5);
            AssertHelpers.AlmostEqualRelative(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqualRelative(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using thin QR factorization with a tall A matrix
        /// using a factored A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingThinQRTallMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var tau = new Complex32[matrix.ColumnCount];
            var r = new Complex32[matrix.ColumnCount*matrix.ColumnCount];
            LinearAlgebraControl.Provider.ThinQRFactor(a, matrix.RowCount, matrix.ColumnCount, r, tau);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.QRSolveFactored(a, r, matrix.RowCount, matrix.ColumnCount, tau, b, 2, x, QRMethod.Thin);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqualRelative(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqualRelative(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqualRelative(test[1, 1], x[3], 5);
        }

        /// <summary>
        /// Can compute the SVD factorization of a square matrix.
        /// </summary>
        [Test]
        public void CanComputeSVDFactorizationOfSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var s = new Complex32[matrix.RowCount];
            var u = new Complex32[matrix.RowCount*matrix.RowCount];
            var vt = new Complex32[matrix.ColumnCount*matrix.ColumnCount];

            LinearAlgebraControl.Provider.SingularValueDecomposition(true, a, matrix.RowCount, matrix.ColumnCount, s, u, vt);

            var w = new DenseMatrix(matrix.RowCount, matrix.ColumnCount);
            for (var index = 0; index < s.Length; index++)
            {
                w[index, index] = s[index];
            }

            var mU = new DenseMatrix(matrix.RowCount, matrix.RowCount, u);
            var mV = new DenseMatrix(matrix.ColumnCount, matrix.ColumnCount, vt);
            var result = mU*w*mV;

            AssertHelpers.AlmostEqualRelative(matrix[0, 0], result[0, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 0], result[1, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[2, 0], result[2, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[0, 1], result[0, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 1], result[1, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[2, 1], result[2, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[0, 2], result[0, 2], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 2], result[1, 2], 5);
            AssertHelpers.AlmostEqualRelative(matrix[2, 2], result[2, 2], 5);
        }

        /// <summary>
        /// Can compute the SVD factorization of a tall matrix.
        /// </summary>
        [Test]
        public void CanComputeSVDFactorizationOfTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var s = new Complex32[matrix.ColumnCount];
            var u = new Complex32[matrix.RowCount*matrix.RowCount];
            var vt = new Complex32[matrix.ColumnCount*matrix.ColumnCount];

            LinearAlgebraControl.Provider.SingularValueDecomposition(true, a, matrix.RowCount, matrix.ColumnCount, s, u, vt);

            var w = new DenseMatrix(matrix.RowCount, matrix.ColumnCount);
            for (var index = 0; index < s.Length; index++)
            {
                w[index, index] = s[index];
            }

            var mU = new DenseMatrix(matrix.RowCount, matrix.RowCount, u);
            var mV = new DenseMatrix(matrix.ColumnCount, matrix.ColumnCount, vt);
            var result = mU*w*mV;

            AssertHelpers.AlmostEqualRelative(matrix[0, 0], result[0, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 0], result[1, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[2, 0], result[2, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[0, 1], result[0, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 1], result[1, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[2, 1], result[2, 1], 4);
        }

        /// <summary>
        /// Can compute the SVD factorization of a wide matrix.
        /// </summary>
        [Test]
        public void CanComputeSVDFactorizationOfWideMatrix()
        {
            var matrix = _matrices["Wide2x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var s = new Complex32[matrix.RowCount];
            var u = new Complex32[matrix.RowCount*matrix.RowCount];
            var vt = new Complex32[matrix.ColumnCount*matrix.ColumnCount];

            LinearAlgebraControl.Provider.SingularValueDecomposition(true, a, matrix.RowCount, matrix.ColumnCount, s, u, vt);

            var w = new DenseMatrix(matrix.RowCount, matrix.ColumnCount);
            for (var index = 0; index < s.Length; index++)
            {
                w[index, index] = s[index];
            }

            var mU = new DenseMatrix(matrix.RowCount, matrix.RowCount, u);
            var mV = new DenseMatrix(matrix.ColumnCount, matrix.ColumnCount, vt);
            var result = mU*w*mV;

            AssertHelpers.AlmostEqualRelative(matrix[0, 0], result[0, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 0], result[1, 0], 5);
            AssertHelpers.AlmostEqualRelative(matrix[0, 1], result[0, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 1], result[1, 1], 5);
            AssertHelpers.AlmostEqualRelative(matrix[0, 2], result[0, 2], 5);
            AssertHelpers.AlmostEqualRelative(matrix[1, 2], result[1, 2], 4);
        }

        /// <summary>
        /// Can solve Ax=b using SVD factorization with a square A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingSVDSquareMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.SvdSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x);

            NotModified(3, 3, a, matrix);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqual(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqual(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqual(mb[2, 0], b[2], 4);
            AssertHelpers.AlmostEqual(mb[0, 1], b[3], 4);
            AssertHelpers.AlmostEqual(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqual(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using SVD factorization with a tall A matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingSvdTallMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.SvdSolve(a, matrix.RowCount, matrix.ColumnCount, b, 2, x);

            NotModified(3, 2, a, matrix);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqual(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqual(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqual(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqual(test[1, 1], x[3], 5);
        }

        /// <summary>
        /// Can solve Ax=b using SVD factorization with a square A matrix
        /// using a factored matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingSvdSquareMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Square3x3"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var s = new Complex32[matrix.RowCount];
            var u = new Complex32[matrix.RowCount*matrix.RowCount];
            var vt = new Complex32[matrix.ColumnCount*matrix.ColumnCount];

            LinearAlgebraControl.Provider.SingularValueDecomposition(true, a, matrix.RowCount, matrix.ColumnCount, s, u, vt);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.SvdSolveFactored(matrix.RowCount, matrix.ColumnCount, s, u, vt, b, 2, x);

            var mx = new DenseMatrix(matrix.ColumnCount, 2, x);
            var mb = matrix*mx;

            AssertHelpers.AlmostEqual(mb[0, 0], b[0], 5);
            AssertHelpers.AlmostEqual(mb[1, 0], b[1], 5);
            AssertHelpers.AlmostEqual(mb[2, 0], b[2], 4);
            AssertHelpers.AlmostEqual(mb[0, 1], b[3], 4);
            AssertHelpers.AlmostEqual(mb[1, 1], b[4], 4);
            AssertHelpers.AlmostEqual(mb[2, 1], b[5], 4);
        }

        /// <summary>
        /// Can solve Ax=b using SVD factorization with a tall A matrix
        /// using a factored matrix.
        /// </summary>
        [Test]
        public void CanSolveUsingSvdTallMatrixOnFactoredMatrix()
        {
            var matrix = _matrices["Tall3x2"];
            var a = new Complex32[matrix.RowCount*matrix.ColumnCount];
            Array.Copy(matrix.Values, a, a.Length);

            var s = new Complex32[matrix.ColumnCount];
            var u = new Complex32[matrix.RowCount*matrix.RowCount];
            var vt = new Complex32[matrix.ColumnCount*matrix.ColumnCount];

            LinearAlgebraControl.Provider.SingularValueDecomposition(true, a, matrix.RowCount, matrix.ColumnCount, s, u, vt);

            var b = new[] {new Complex32(1.0f, 0.0f), 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
            var x = new Complex32[matrix.ColumnCount*2];
            LinearAlgebraControl.Provider.SvdSolveFactored(matrix.RowCount, matrix.ColumnCount, s, u, vt, b, 2, x);

            var mb = new DenseMatrix(matrix.RowCount, 2, b);
            var test = (matrix.Transpose()*matrix).Inverse()*matrix.Transpose()*mb;

            AssertHelpers.AlmostEqual(test[0, 0], x[0], 5);
            AssertHelpers.AlmostEqual(test[1, 0], x[1], 5);
            AssertHelpers.AlmostEqual(test[0, 1], x[2], 5);
            AssertHelpers.AlmostEqual(test[1, 1], x[3], 5);
        }

        [TestCase("Wide10x50000", "Tall50000x10")]
        [TestCase("Square1000x1000", "Square1000x1000")]
        [Explicit]
        public void IsMatrixMultiplicationPerformant(string leftMatrixKey, string rightMatrixKey)
        {
            var leftMatrix = _matrices[leftMatrixKey];
            var rightMatrix = _matrices[rightMatrixKey];
            var result = leftMatrix*rightMatrix;
            Assert.That(result, Is.Not.Null);
        }

        /// <summary>
        /// Checks to see if a matrix and array contain the same values.
        /// </summary>
        /// <param name="rows">number of rows.</param>
        /// <param name="columns">number of columns.</param>
        /// <param name="array">array to check.</param>
        /// <param name="matrix">matrix to check against.</param>
        static void NotModified(int rows, int columns, IList<Complex32> array, Matrix<Complex32> matrix)
        {
            var index = 0;
            for (var col = 0; col < columns; col++)
            {
                for (var row = 0; row < rows; row++)
                {
                    Assert.AreEqual(array[index++], matrix[row, col]);
                }
            }
        }
    }
}
