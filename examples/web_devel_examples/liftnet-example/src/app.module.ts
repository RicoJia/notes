// A must for bootstrapping controller into the app module
import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { FakeCarModuleModule } from './fake_car_module/fake_car_module.module';

// Configs
// RICO: envConfig: I still don't know what it is.
// import envConfig, { AppEnv, IConfiguration } from './config/env.config';
import { ConfigModule, ConfigService } from '@nestjs/config';

@Module({
  imports: [
    FakeCarModuleModule,
    ConfigModule.forRoot({
    //   isGlobal: true,
    //   cache: true
        // load: [envConfig],
        // envFilePath: process.env.APP_ENV === AppEnv.TEST ? DOT_ENV_TEST : undefined,
        })
    ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
