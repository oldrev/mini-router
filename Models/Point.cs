using System;
using System.Collections.Generic;
using System.Text;
using System.Numerics;
using System.Runtime.Intrinsics;

namespace MinimalRouter.Models;

readonly struct Point
{
    readonly Vector128<double> vec;
}
