var webppl = require("/usr/lib/node_modules/webppl/src/main.js");
var args = require("/usr/lib/node_modules/webppl/src/args.js");
args.makeGlobal(__filename, process.argv.slice(2));
var __runner__ = util.trampolineRunners.cli();
function topK(s, x) {
  console.log(x);
};
var main = (function (_globalCurrentAddress) {
    return function (p) {
        return function (runTrampoline) {
            return function (s, k, a) {
                runTrampoline(function () {
                    return p(s, k, a);
                });
            };
        };
    }(function (globalStore, _k0, _address0) {
        var _currentAddress = _address0;
        _addr.save(_globalCurrentAddress, _address0);
        var Bernoulli = dists.makeBernoulli;
        var Beta = dists.makeBeta;
        var Binomial = dists.makeBinomial;
        var Categorical = dists.makeCategorical;
        var Cauchy = dists.makeCauchy;
        var Delta = dists.makeDelta;
        var DiagCovGaussian = dists.makeDiagCovGaussian;
        var Dirichlet = dists.makeDirichlet;
        var Discrete = dists.makeDiscrete;
        var Exponential = dists.makeExponential;
        var Gamma = dists.makeGamma;
        var Gaussian = dists.makeGaussian;
        var ImproperUniform = dists.makeImproperUniform;
        var IspNormal = dists.makeIspNormal;
        var KDE = dists.makeKDE;
        var Laplace = dists.makeLaplace;
        var LogisticNormal = dists.makeLogisticNormal;
        var LogitNormal = dists.makeLogitNormal;
        var Marginal = dists.makeMarginal;
        var Mixture = dists.makeMixture;
        var Multinomial = dists.makeMultinomial;
        var MultivariateBernoulli = dists.makeMultivariateBernoulli;
        var MultivariateGaussian = dists.makeMultivariateGaussian;
        var Poisson = dists.makePoisson;
        var RandomInteger = dists.makeRandomInteger;
        var SampleBasedMarginal = dists.makeSampleBasedMarginal;
        var TensorGaussian = dists.makeTensorGaussian;
        var TensorLaplace = dists.makeTensorLaplace;
        var Uniform = dists.makeUniform;
        var flip = function flip(globalStore, _k323, _address26, p) {
            var _currentAddress = _address26;
            _addr.save(_globalCurrentAddress, _address26);
            var _k326 = function (globalStore, _result325) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                var params = { p: _result325 };
                return function () {
                    return Bernoulli(globalStore, function (globalStore, _result324) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        return function () {
                            return sample(globalStore, _k323, _address26.concat('_51'), _result324);
                        };
                    }, _address26.concat('_50'), params);
                };
            };
            return function () {
                return ad.scalar.pneq(p, undefined) ? _k326(globalStore, p) : _k326(globalStore, 0.5);
            };
        };
        var condition = function condition(globalStore, _k148, _address120, bool) {
            var _currentAddress = _address120;
            _addr.save(_globalCurrentAddress, _address120);
            var _k150 = function (globalStore, _result149) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                return function () {
                    return factor(globalStore, _k148, _address120.concat('_150'), _result149);
                };
            };
            return function () {
                return bool ? _k150(globalStore, 0) : _k150(globalStore, ad.scalar.neg(Infinity));
            };
        };
        var error = function error(globalStore, _k147, _address121, msg) {
            var _currentAddress = _address121;
            _addr.save(_globalCurrentAddress, _address121);
            return function () {
                return _k147(globalStore, util.error(msg));
            };
        };
        var SampleGuide = function SampleGuide(globalStore, _k143, _address125, wpplFn, options) {
            var _currentAddress = _address125;
            _addr.save(_globalCurrentAddress, _address125);
            return function () {
                return ForwardSample(globalStore, _k143, _address125.concat('_154'), wpplFn, _.assign({ guide: !0 }, _.omit(options, 'guide')));
            };
        };
        var OptimizeThenSample = function OptimizeThenSample(globalStore, _k141, _address126, wpplFn, options) {
            var _currentAddress = _address126;
            _addr.save(_globalCurrentAddress, _address126);
            return function () {
                return Optimize(globalStore, function (globalStore, _dummy142) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    var opts = _.pick(options, 'samples', 'onlyMAP', 'verbose');
                    return function () {
                        return SampleGuide(globalStore, _k141, _address126.concat('_156'), wpplFn, opts);
                    };
                }, _address126.concat('_155'), wpplFn, _.omit(options, 'samples', 'onlyMAP'));
            };
        };
        var DefaultInfer = function DefaultInfer(globalStore, _k131, _address127, wpplFn, options) {
            var _currentAddress = _address127;
            _addr.save(_globalCurrentAddress, _address127);
            var _dummy140 = util.mergeDefaults(options, {}, 'Infer');
            var maxEnumTreeSize = 200000;
            var minSampleRate = 250;
            var samples = 1000;
            return function () {
                return Enumerate(globalStore, function (globalStore, enumResult) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    var _k139 = function (globalStore, _dummy138) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        var _dummy137 = console.log('Using "rejection"');
                        return function () {
                            return Rejection(globalStore, function (globalStore, rejResult) {
                                _addr.save(_globalCurrentAddress, _currentAddress);
                                return function () {
                                    return rejResult instanceof Error ? function (globalStore, _dummy136) {
                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                        return function () {
                                            return CheckSampleAfterFactor(globalStore, function (globalStore, hasSampleAfterFactor) {
                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                var _k134 = function (globalStore, _dummy133) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    var _dummy132 = console.log('Using "MCMC"');
                                                    return function () {
                                                        return MCMC(globalStore, _k131, _address127.concat('_163'), wpplFn, { samples: samples });
                                                    };
                                                };
                                                return function () {
                                                    return hasSampleAfterFactor ? function (globalStore, _dummy135) {
                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                        return function () {
                                                            return SMC(globalStore, function (globalStore, smcResult) {
                                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                                return function () {
                                                                    return dists.isDist(smcResult) ? _k131(globalStore, smcResult) : smcResult instanceof Error ? _k134(globalStore, console.log(ad.scalar.add(smcResult.message, '..quit SMC'))) : error(globalStore, _k134, _address127.concat('_162'), 'Invalid return value from SMC');
                                                                };
                                                            }, _address127.concat('_161'), wpplFn, {
                                                                throwOnError: !1,
                                                                particles: samples
                                                            });
                                                        };
                                                    }(globalStore, console.log('Using "SMC" (interleaving samples and factors detected)')) : _k134(globalStore, undefined);
                                                };
                                            }, _address127.concat('_160'), wpplFn);
                                        };
                                    }(globalStore, console.log(ad.scalar.add(rejResult.message, '..quit rejection'))) : dists.isDist(rejResult) ? _k131(globalStore, rejResult) : error(globalStore, _k131, _address127.concat('_164'), 'Invalid return value from rejection');
                                };
                            }, _address127.concat('_159'), wpplFn, {
                                minSampleRate: minSampleRate,
                                throwOnError: !1,
                                samples: samples
                            });
                        };
                    };
                    return function () {
                        return dists.isDist(enumResult) ? _k131(globalStore, enumResult) : enumResult instanceof Error ? _k139(globalStore, console.log(ad.scalar.add(enumResult.message, '..quit enumerate'))) : error(globalStore, _k139, _address127.concat('_158'), 'Invalid return value from enumerate');
                    };
                }, _address127.concat('_157'), wpplFn, {
                    maxEnumTreeSize: maxEnumTreeSize,
                    maxRuntimeInMS: 5000,
                    throwOnError: !1,
                    strategy: 'depthFirst'
                });
            };
        };
        var Infer = function Infer(globalStore, _k124, _address128, options, maybeFn) {
            var _currentAddress = _address128;
            _addr.save(_globalCurrentAddress, _address128);
            var _k130 = function (globalStore, wpplFn) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                var _k129 = function (globalStore, _dummy128) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    var methodMap = {
                        SMC: SMC,
                        MCMC: MCMC,
                        PMCMC: PMCMC,
                        asyncPF: AsyncPF,
                        rejection: Rejection,
                        enumerate: Enumerate,
                        incrementalMH: IncrementalMH,
                        forward: ForwardSample,
                        optimize: OptimizeThenSample,
                        defaultInfer: DefaultInfer
                    };
                    var _k127 = function (globalStore, methodName) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        var _k126 = function (globalStore, _dummy125) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            var method = methodMap[methodName];
                            return function () {
                                return method(globalStore, _k124, _address128.concat('_167'), wpplFn, _.omit(options, 'method', 'model'));
                            };
                        };
                        return function () {
                            return _.has(methodMap, methodName) ? _k126(globalStore, undefined) : function (globalStore, methodNames) {
                                _addr.save(_globalCurrentAddress, _currentAddress);
                                var msg = ad.scalar.add(ad.scalar.add(ad.scalar.add(ad.scalar.add('Infer: \'', methodName), '\' is not a valid method. The following methods are available: '), methodNames.join(', ')), '.');
                                return function () {
                                    return error(globalStore, _k126, _address128.concat('_166'), msg);
                                };
                            }(globalStore, _.keys(methodMap));
                        };
                    };
                    return function () {
                        return options.method ? _k127(globalStore, options.method) : _k127(globalStore, 'defaultInfer');
                    };
                };
                return function () {
                    return _.isFunction(wpplFn) ? _k129(globalStore, undefined) : error(globalStore, _k129, _address128.concat('_165'), 'Infer: a model was not specified.');
                };
            };
            return function () {
                return util.isObject(options) ? maybeFn ? _k130(globalStore, maybeFn) : _k130(globalStore, options.model) : _k130(globalStore, options);
            };
        };
        var model = function model(globalStore, _k1, _address163) {
            var _currentAddress = _address163;
            _addr.save(_globalCurrentAddress, _address163);
            return function () {
                return flip(globalStore, function (globalStore, coin) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return condition(globalStore, _k1, _address163.concat('_253'), ad.scalar.eq(coin, !0));
                    };
                }, _address163.concat('_252'), 0.5);
            };
        };
        return function () {
            return Infer(globalStore, _k0, _address0.concat('_254'), model);
        };
    });
});

webppl.runEvaled(main, __runner__, {}, {}, topK, '');