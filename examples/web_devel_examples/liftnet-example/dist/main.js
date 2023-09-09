"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const core_1 = require("@nestjs/core");
const swagger_1 = require("@nestjs/swagger");
const platform_fastify_1 = require("@nestjs/platform-fastify");
const app_config_1 = require("./config/app.config");
const app_module_1 = require("./app.module");
/**
 * Rico: using async function implicitly returning a promise
 */
async function bootstrap() {
    // getting AppModule here
    const app = await core_1.NestFactory.create(app_module_1.AppModule, new platform_fastify_1.FastifyAdapter());
    // In prod
    await (0, app_config_1.configureApp)(app);
    // below is in configureSwagger(app);
    //   const config = app.get(ConfigService).getOrThrow<IConfiguration>(CONFIG_NAMESPACE);
    const config = new swagger_1.DocumentBuilder()
        .setTitle('Rico\'s NestJS API')
        .setDescription('The API description')
        .setVersion('1.0')
        .addTag('nestjs')
        .build();
    const document = swagger_1.SwaggerModule.createDocument(app, config);
    // this defines http://localhost:3000/api-doc. Adding swagger doc to the app
    swagger_1.SwaggerModule.setup('api-doc', app, document);
    await app.listen(3000);
}
bootstrap().catch((e) => {
    console.error(e);
});
