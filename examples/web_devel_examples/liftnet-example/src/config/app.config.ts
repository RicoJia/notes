import { NestFastifyApplication } from '@nestjs/platform-fastify';
// config service must be exported in app.module?
import { ConfigService } from '@nestjs/config';
/*@src is defined in tsconfig.json*/
import { CONFIG_NAMESPACE, DEFAULT_API_VERSION } from '@src/core/constants';
import { AppEnv, IConfiguration } from './env.config';

/**
 * Putting application wide configs here.
 * 
 */
export async function configureApp(app: NestFastifyApplication): Promise<void> {
//   const config = app.get<ConfigService>(ConfigService).getOrThrow<IConfiguration>(CONFIG_NAMESPACE);
//   console.log("rico: ", config);
}