// import { CONFIG_NAMESPACE, DEFAULT_APP_HOST, DEFAULT_APP_PORT } from '@src/core/constants';
// export default registerAs(CONFIG_NAMESPACE, (): IConfiguration => {

// });

export interface IConfiguration {
  readonly app: {
    readonly env: AppEnv;
    readonly host: string;
    readonly port: number;
    readonly isDeployed: boolean;
    readonly logLevel: LogLevel;
    readonly modbusDebug: boolean;
  };
}

export enum AppEnv {
  LOCAL = 'LOCAL',
  TEST = 'TEST',
  DEV = 'DEV',
  STAGE = 'STAGE',
  PROD = 'PROD',
}

export enum LogLevel {
  INFO = 'info',
  DEBUG = 'debug',
  TRACE = 'trace',
}