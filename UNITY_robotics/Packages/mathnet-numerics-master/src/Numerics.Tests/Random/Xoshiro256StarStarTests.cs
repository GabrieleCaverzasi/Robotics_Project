// <copyright file="Xoshiro256StarStarTests.cs" company="Math.NET">
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

using MathNet.Numerics.Random;
using NUnit.Framework;

namespace MathNet.Numerics.Tests.Random
{
    /// <summary>
    /// Test for xoshiro256** pseudo random number generator.
    /// </summary>
    [TestFixture, Category("Random")]
    public class Xoshiro256StarStarTests : RandomTests
    {
        /// <summary>
        /// Initializes a new instance of the Wh2006Tests class.
        /// </summary>
        public Xoshiro256StarStarTests() : base(typeof (Xoshiro256StarStar))
        {
        }

        [Test]
        public void StaticSamplesConsistent()
        {
            Assert.That(Xoshiro256StarStar.Doubles(1000, 1), Is.EqualTo(new Xoshiro256StarStar(1).NextDoubles(1000)).Within(1e-12).AsCollection);
        }
    }
}
