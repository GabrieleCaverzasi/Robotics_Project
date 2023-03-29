using NUnit.Framework;
using static Unity.Mathematics.math;
using Burst.Compiler.IL.Tests;

namespace Unity.Mathematics.Tests
{
    [TestFixture]
    public partial class TestMath
    {
        [TestCompiler]
        public unsafe void hash_blob()
        {
            byte[] testData = {
                0x0d, 0x26, 0x1c, 0xeb, 0x56, 0x3a, 0x9c, 0x18, 0x93, 0xb6,
                0xc1, 0x99, 0x5e, 0x04, 0x92, 0x4f, 0x6e, 0xb7, 0x42, 0x53,
                0x23, 0xcf, 0xe3, 0xbf, 0x16, 0x64, 0x79, 0x08, 0xc1, 0x01,
                0x43, 0x89, 0x73, 0x8f, 0x76, 0x22, 0x0c, 0xee, 0x9b, 0x80,
                0x31, 0x83, 0xce, 0x33, 0x8b, 0xc7, 0x3f, 0x94, 0x33
            };

            uint[] resultsWithZeroSeed =
            {
                0x02cc5d05, 0x376a5b3f, 0x8ae13198, 0xf5b14d72, 0xcc7ddc84,
                0x763e5905, 0x58759392, 0x6bccbd00, 0x7d0f80c8, 0xef01ae48,
                0xe40aa3ad, 0x805e04ad, 0xf98a471c, 0xdd960ac1, 0x76a71750,
                0xd35e2baa, 0x0219f7da, 0xd5bd1fbd, 0x5f28c87e, 0xfe6f7995,
                0x69a43ac5, 0xec1a1a15, 0x7ae9f103, 0x8d04a688, 0xce3b35be,
                0x6b3f040c, 0xd5ea2e8c, 0x6989e79c, 0x8772f1fb, 0x0d7b7bf6,
                0x0796214b, 0x98ea65c4, 0x3884dd82, 0x59632484, 0x91c92822,
                0x72d28404, 0x167061b0, 0x32adc2ef, 0xbac3e672, 0xd39936b7,
                0x94e2c154, 0x8b4ff46c, 0x68976fd4, 0x04bee59a, 0x2ed62c69,
                0xabee69fd, 0xda11e266, 0xcebd9d38, 0x28eea5fd, 0x0210d6ee
            };


            uint[] resultsWith_0xeb69cf40_seed =
            {
                0x12c3b280, 0x9bc1f68d, 0x2f900b51, 0xdb77e20e, 0xf6e8f561,
                0x3f3f72f6, 0x15f9700f, 0x28beb671, 0xceece5e1, 0x7a9b5c81,
                0x62a84642, 0xf75666af, 0x8939f8ca, 0x6b84792b, 0x527ee836,
                0xa0782090, 0xce6b9926, 0xa608c73b, 0xa08ee6a3, 0xab77a6b2,
                0x4e174e68, 0x596f10d5, 0xa14c4d85, 0x509ab88d, 0xd14698a4,
                0xbaad2308, 0xbd04c3df, 0x5715fed1, 0x5cf23a74, 0xd4e844f9,
                0x166dcfba, 0xe3495e37, 0x1b18b2b3, 0x2e8c2aab, 0x40993321,
                0x4b84998a, 0xd062937d, 0x1b2c9f7b, 0x8a613ed3, 0x70d71291,
                0x1af38ea4, 0x460cb7e9, 0xf806e3a9, 0xec9b6b2e, 0x9972a843,
                0x1ff06d2a, 0xe8c5007b, 0x37d8fc40, 0x0dc4f639, 0x36edff4f
            };
            fixed (byte* p = testData)
            {
                for (int i = 0; i < 50; ++i)
                {
                    TestUtils.AreEqual(hash(p, i, 0), resultsWithZeroSeed[i]);
                }

                for (int i = 0; i < 50; ++i)
                {
                    TestUtils.AreEqual(hash(p, i, 0xeb69cf40), resultsWith_0xeb69cf40_seed[i]);
                }
            }
        }
    }
}
