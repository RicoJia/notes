"use strict";
// import { CONFIG_NAMESPACE, DEFAULT_APP_HOST, DEFAULT_APP_PORT } from '@src/core/constants';
// export default registerAs(CONFIG_NAMESPACE, (): IConfiguration => {
Object.defineProperty(exports, "__esModule", { value: true });
exports.LogLevel = exports.AppEnv = void 0;
var AppEnv;
(function (AppEnv) {
    AppEnv["LOCAL"] = "LOCAL";
    AppEnv["TEST"] = "TEST";
    AppEnv["DEV"] = "DEV";
    AppEnv["STAGE"] = "STAGE";
    AppEnv["PROD"] = "PROD";
})(AppEnv || (exports.AppEnv = AppEnv = {}));
var LogLevel;
(function (LogLevel) {
    LogLevel["INFO"] = "info";
    LogLevel["DEBUG"] = "debug";
    LogLevel["TRACE"] = "trace";
})(LogLevel || (exports.LogLevel = LogLevel = {}));
