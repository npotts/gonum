// Copyright ©2018 The Gonum Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package dualquat

import "gonum.org/v1/gonum/num/quat"

// First derivatives:

func dSin(x quat.Quat) quat.Quat  { return quat.Cos(x) }
func dCos(x quat.Quat) quat.Quat  { return quat.Scale(-1, quat.Sin(x)) }
func dTan(x quat.Quat) quat.Quat  { return quat.Mul(sec(x), sec(x)) }
func dAsin(x quat.Quat) quat.Quat { return quat.Inv(quat.Sqrt(subRealQuat(1, quat.Mul(x, x)))) }
func dAcos(x quat.Quat) quat.Quat {
	return quat.Scale(-1, quat.Inv(quat.Sqrt(subRealQuat(1, quat.Mul(x, x)))))
}
func dAtan(x quat.Quat) quat.Quat { return quat.Inv(addRealQuat(1, quat.Mul(x, x))) }

func dSinh(x quat.Quat) quat.Quat  { return quat.Cosh(x) }
func dCosh(x quat.Quat) quat.Quat  { return quat.Sinh(x) }
func dTanh(x quat.Quat) quat.Quat  { return quat.Mul(sech(x), sech(x)) }
func dAsinh(x quat.Quat) quat.Quat { return quat.Inv(quat.Sqrt(addQuatReal(quat.Mul(x, x), 1))) }
func dAcosh(x quat.Quat) quat.Quat {
	return quat.Inv(quat.Mul(quat.Sqrt(subQuatReal(x, 1)), quat.Sqrt(addQuatReal(x, 1))))
}
func dAtanh(x quat.Quat) quat.Quat { return quat.Inv(subRealQuat(1, quat.Mul(x, x))) }

func dExp(x quat.Quat) quat.Quat    { return quat.Exp(x) }
func dLog(x quat.Quat) quat.Quat    { return quat.Inv(x) }
func dPow(x, y quat.Quat) quat.Quat { return quat.Mul(y, quat.Pow(x, subQuatReal(y, 1))) }
func dSqrt(x quat.Quat) quat.Quat   { return quat.Scale(0.5, quat.Sqrt(x)) }
func dInv(x quat.Quat) quat.Quat    { return quat.Scale(-1, quat.Inv(quat.Mul(x, x))) }

// Helpers:

func sec(x quat.Quat) quat.Quat  { return quat.Inv(quat.Cos(x)) }
func sech(x quat.Quat) quat.Quat { return quat.Inv(quat.Cosh(x)) }

/*

var dualTests = []struct {
	name   string
	x      []float64
	fnDual func(x Number) Number
	fn     func(x float64) float64
	dFn    func(x float64) float64
}{
	{
		name:   "sin",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Sin,
		fn:     math.Sin,
		dFn:    dSin,
	},
	{
		name:   "cos",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Cos,
		fn:     math.Cos,
		dFn:    dCos,
	},
	{
		name:   "tan",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Tan,
		fn:     math.Tan,
		dFn:    dTan,
	},
	{
		name:   "sinh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Sinh,
		fn:     math.Sinh,
		dFn:    dSinh,
	},
	{
		name:   "cosh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Cosh,
		fn:     math.Cosh,
		dFn:    dCosh,
	},
	{
		name:   "tanh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Tanh,
		fn:     math.Tanh,
		dFn:    dTanh,
	},

	{
		name:   "asin",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Asin,
		fn:     math.Asin,
		dFn:    dAsin,
	},
	{
		name:   "acos",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Acos,
		fn:     math.Acos,
		dFn:    dAcos,
	},
	{
		name:   "atan",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Atan,
		fn:     math.Atan,
		dFn:    dAtan,
	},
	{
		name:   "asinh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Asinh,
		fn:     math.Asinh,
		dFn:    dAsinh,
	},
	{
		name:   "acosh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Acosh,
		fn:     math.Acosh,
		dFn:    dAcosh,
	},
	{
		name:   "atanh",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Atanh,
		fn:     math.Atanh,
		dFn:    dAtanh,
	},

	{
		name:   "exp",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Exp,
		fn:     math.Exp,
		dFn:    dExp,
	},
	{
		name:   "log",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Log,
		fn:     math.Log,
		dFn:    dLog,
	},
	{
		name:   "inv",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Inv,
		fn:     func(x float64) float64 { return 1 / x },
		dFn:    dInv,
	},
	{
		name:   "sqrt",
		x:      []float64{math.NaN(), math.Inf(-1), -3, -2, -1, -0.5, negZero, 0, 0.5, 1, 2, 3, math.Inf(1)},
		fnDual: Sqrt,
		fn:     math.Sqrt,
		dFn:    dSqrt,
	},
}

func TestDual(t *testing.T) {
	const tol = 1e-15
	for _, test := range dualTests {
		for _, x := range test.x {
			fxDual := test.fnDual(Number{Real: x, Emag: 1})
			fx := test.fn(x)
			dFx := test.dFn(x)
			if !same(fxDual.Real, fx, tol) {
				t.Errorf("unexpected %s(%v): got:%v want:%v", test.name, x, fxDual.Real, fx)
			}
			if !same(fxDual.Emag, dFx, tol) {
				t.Errorf("unexpected %s'(%v): got:%v want:%v", test.name, x, fxDual.Emag, dFx)
			}
		}
	}
}

var powRealTests = []struct {
	d    Number
	p    float64
	want Number
}{
	// PowReal(NaN+xϵ, ±0) = 1+NaNϵ for any x
	{d: Number{Real: math.NaN(), Emag: 0}, p: 0, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 0}, p: negZero, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 1}, p: 0, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 2}, p: negZero, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 3}, p: 0, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 1}, p: negZero, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 2}, p: 0, want: Number{Real: 1, Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 3}, p: negZero, want: Number{Real: 1, Emag: math.NaN()}},

	// PowReal(x, ±0) = 1 for any x
	{d: Number{Real: 0, Emag: 0}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: negZero, Emag: 0}, p: negZero, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: math.Inf(1), Emag: 0}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: math.Inf(-1), Emag: 0}, p: negZero, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 0, Emag: 1}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: negZero, Emag: 1}, p: negZero, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: math.Inf(1), Emag: 1}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: math.Inf(-1), Emag: 1}, p: negZero, want: Number{Real: 1, Emag: 0}},

	// PowReal(1+xϵ, y) = (1+xyϵ) for any y
	{d: Number{Real: 1, Emag: 0}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 0}, p: 1, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 0}, p: 2, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 0}, p: 3, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 1}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 1}, p: 1, want: Number{Real: 1, Emag: 1}},
	{d: Number{Real: 1, Emag: 1}, p: 2, want: Number{Real: 1, Emag: 2}},
	{d: Number{Real: 1, Emag: 1}, p: 3, want: Number{Real: 1, Emag: 3}},
	{d: Number{Real: 1, Emag: 2}, p: 0, want: Number{Real: 1, Emag: 0}},
	{d: Number{Real: 1, Emag: 2}, p: 1, want: Number{Real: 1, Emag: 2}},
	{d: Number{Real: 1, Emag: 2}, p: 2, want: Number{Real: 1, Emag: 4}},
	{d: Number{Real: 1, Emag: 2}, p: 3, want: Number{Real: 1, Emag: 6}},

	// PowReal(x, 1) = x for any x
	{d: Number{Real: 0, Emag: 0}, p: 1, want: Number{Real: 0, Emag: 0}},
	{d: Number{Real: negZero, Emag: 0}, p: 1, want: Number{Real: negZero, Emag: 0}},
	{d: Number{Real: 0, Emag: 1}, p: 1, want: Number{Real: 0, Emag: 1}},
	{d: Number{Real: negZero, Emag: 1}, p: 1, want: Number{Real: negZero, Emag: 1}},
	{d: Number{Real: math.NaN(), Emag: 0}, p: 1, want: Number{Real: math.NaN(), Emag: 0}},
	{d: Number{Real: math.NaN(), Emag: 1}, p: 1, want: Number{Real: math.NaN(), Emag: 1}},
	{d: Number{Real: math.NaN(), Emag: 2}, p: 1, want: Number{Real: math.NaN(), Emag: 2}},

	// PowReal(NaN+xϵ, y) = NaN+NaNϵ
	{d: Number{Real: math.NaN(), Emag: 0}, p: 2, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 0}, p: 3, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 1}, p: 2, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 1}, p: 3, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 2}, p: 2, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: math.NaN(), Emag: 2}, p: 3, want: Number{Real: math.NaN(), Emag: math.NaN()}},

	// PowReal(x, NaN) = NaN+NaNϵ
	{d: Number{Real: 0, Emag: 0}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 2, Emag: 0}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 0}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 0, Emag: 1}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 2, Emag: 1}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 1}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 0, Emag: 2}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 2, Emag: 2}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 2}, p: math.NaN(), want: Number{Real: math.NaN(), Emag: math.NaN()}},

	// Handled by math.Pow tests:
	//
	// Pow(±0, y) = ±Inf for y an odd integer < 0
	// Pow(±0, -Inf) = +Inf
	// Pow(±0, +Inf) = +0
	// Pow(±0, y) = +Inf for finite y < 0 and not an odd integer
	// Pow(±0, y) = ±0 for y an odd integer > 0
	// Pow(±0, y) = +0 for finite y > 0 and not an odd integer
	// Pow(-1, ±Inf) = 1

	// PowReal(x+0ϵ, +Inf) = +Inf+NaNϵ for |x| > 1
	{d: Number{Real: 2, Emag: 0}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 0}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.NaN()}},

	// PowReal(x+yϵ, +Inf) = +Inf for |x| > 1
	{d: Number{Real: 2, Emag: 1}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.Inf(1)}},
	{d: Number{Real: 3, Emag: 1}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.Inf(1)}},
	{d: Number{Real: 2, Emag: 2}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.Inf(1)}},
	{d: Number{Real: 3, Emag: 2}, p: math.Inf(1), want: Number{Real: math.Inf(1), Emag: math.Inf(1)}},

	// PowReal(x, -Inf) = +0+NaNϵ for |x| > 1
	{d: Number{Real: 2, Emag: 0}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 0}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 2, Emag: 1}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 1}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 2, Emag: 2}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 3, Emag: 2}, p: math.Inf(-1), want: Number{Real: 0, Emag: math.NaN()}},

	// PowReal(x+yϵ, +Inf) = +0+NaNϵ for |x| < 1
	{d: Number{Real: 0.1, Emag: 0}, p: math.Inf(1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 0.1, Emag: 0.1}, p: math.Inf(1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 0.2, Emag: 0.2}, p: math.Inf(1), want: Number{Real: 0, Emag: math.NaN()}},
	{d: Number{Real: 0.5, Emag: 0.5}, p: math.Inf(1), want: Number{Real: 0, Emag: math.NaN()}},

	// PowReal(x+0ϵ, -Inf) = +Inf+NaNϵ for |x| < 1
	{d: Number{Real: 0.1, Emag: 0}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.NaN()}},
	{d: Number{Real: 0.2, Emag: 0}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.NaN()}},

	// PowReal(x, -Inf) = +Inf-Infϵ for |x| < 1
	{d: Number{Real: 0.1, Emag: 0.1}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.2, Emag: 0.1}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.1, Emag: 0.2}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.2, Emag: 0.2}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.1, Emag: 1}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.2, Emag: 1}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.1, Emag: 2}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},
	{d: Number{Real: 0.2, Emag: 2}, p: math.Inf(-1), want: Number{Real: math.Inf(1), Emag: math.Inf(-1)}},

	// Handled by math.Pow tests:
	//
	// Pow(+Inf, y) = +Inf for y > 0
	// Pow(+Inf, y) = +0 for y < 0
	// Pow(-Inf, y) = Pow(-0, -y)

	// PowReal(x, y) = NaN+NaNϵ for finite x < 0 and finite non-integer y
	{d: Number{Real: -1, Emag: -1}, p: 0.5, want: Number{Real: math.NaN(), Emag: math.NaN()}},
	{d: Number{Real: -1, Emag: 2}, p: 0.5, want: Number{Real: math.NaN(), Emag: math.NaN()}},
}

func TestPowReal(t *testing.T) {
	const tol = 1e-15
	for _, test := range powRealTests {
		got := PowReal(test.d, test.p)
		if !sameDual(got, test.want, tol) {
			t.Errorf("unexpected PowReal(%v, %v): got:%v want:%v", test.d, test.p, got, test.want)
		}
	}
}

func sameDual(a, b Number, tol float64) bool {
	return same(a.Real, b.Real, tol) && same(a.Emag, b.Emag, tol)
}

func same(a, b, tol float64) bool {
	return (math.IsNaN(a) && math.IsNaN(b)) || floats.EqualWithinAbsOrRel(a, b, tol, tol)
}
*/
