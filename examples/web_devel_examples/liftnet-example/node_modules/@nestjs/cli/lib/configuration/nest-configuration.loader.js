"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.NestConfigurationLoader = void 0;
const defaults_1 = require("./defaults");
class NestConfigurationLoader {
    constructor(reader) {
        this.reader = reader;
    }
    async load(name) {
        const content = name
            ? await this.reader.read(name)
            : await this.reader.readAnyOf([
                'nest-cli.json',
                '.nestcli.json',
                '.nest-cli.json',
                'nest.json',
            ]);
        if (!content) {
            return defaults_1.defaultConfiguration;
        }
        const fileConfig = JSON.parse(content);
        if (fileConfig.compilerOptions) {
            return {
                ...defaults_1.defaultConfiguration,
                ...fileConfig,
                compilerOptions: {
                    ...defaults_1.defaultConfiguration.compilerOptions,
                    ...fileConfig.compilerOptions,
                },
            };
        }
        return {
            ...defaults_1.defaultConfiguration,
            ...fileConfig,
        };
    }
}
exports.NestConfigurationLoader = NestConfigurationLoader;
