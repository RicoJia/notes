export interface PluginOptions {
    dtoFileNameSuffix?: string | string[];
    controllerFileNameSuffix?: string | string[];
    classValidatorShim?: boolean;
    dtoKeyOfComment?: string;
    controllerKeyOfComment?: string;
    introspectComments?: boolean;
    readonly?: boolean;
    pathToSource?: string;
    debug?: boolean;
}
export declare const mergePluginOptions: (options?: Record<string, any>) => PluginOptions;
