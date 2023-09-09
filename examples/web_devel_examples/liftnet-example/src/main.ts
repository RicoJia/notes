import { NestFactory } from '@nestjs/core';
import { SwaggerModule, DocumentBuilder } from '@nestjs/swagger';
import { FastifyAdapter, NestFastifyApplication } from '@nestjs/platform-fastify';

import { configureApp } from './config/app.config';
import { AppModule } from './app.module';
import { ConfigService } from '@nestjs/config';
import { IConfiguration } from './config/env.config';
import { CONFIG_NAMESPACE } from './core/constants';
/**
 * Rico: using async function implicitly returning a promise
 */
async function bootstrap(): Promise<void> {
  // getting AppModule here
  const app = await NestFactory.create<NestFastifyApplication>(AppModule, new FastifyAdapter());

  // In prod
  await configureApp(app);
  // below is in configureSwagger(app);
//   const config = app.get(ConfigService).getOrThrow<IConfiguration>(CONFIG_NAMESPACE);
  const config = new DocumentBuilder()
    .setTitle('Rico\'s NestJS API')
    .setDescription('The API description')
    .setVersion('1.0')
    .addTag('nestjs')
    .build();

  const document = SwaggerModule.createDocument(app, config);
  // this defines http://localhost:3000/api-doc. Adding swagger doc to the app
  SwaggerModule.setup('api-doc', app, document);

  await app.listen(3000);
}
bootstrap().catch((e) => {
    console.error(e);
})
