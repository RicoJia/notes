import { NestFactory } from '@nestjs/core';
import { SwaggerModule, DocumentBuilder } from '@nestjs/swagger';
import { AppModule } from './app.module';

async function bootstrap() {
  // getting AppModule here
  const app = await NestFactory.create(AppModule);

  // In prod
  //   await configureApp(app);
  // below is in configureSwagger(app);
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
bootstrap();
