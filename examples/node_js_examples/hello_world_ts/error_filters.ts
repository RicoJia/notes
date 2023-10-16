import { ArgumentsHost, Catch, ExceptionFilter, HttpException, HttpStatus } from '@nestjs/common';
import { FastifyReply } from 'fastify';

/**
 - HTTP request: 
    - header, 
    - body (actual data being sent, in POST, or PUT), 
    - method (GET, etc.)
    - URL: url being accessed
    - Cookies: any strings sent by the client
    - params: URL params, query string params
    - client info: IP address
- HTTP response
    - header
    - status code
    - body
    - msg
- session data
 */
@Catch()
export class GenericReturnInResponseFilter implements ExceptionFilter {
  public catch(exception: Error, host: ArgumentsHost): void {
    // get status code
    const statusCode = exception instanceof HttpException ? exception.getStatus() : HttpStatus.INTERNAL_SERVER_ERROR;

    // Get Error type:
    const errorType = exception.constructor.name;

    const messageBody = {
      statusCode: statusCode,
      errorType: errorType,
      // some exceptions, like BadRequestException, has useful information in exception.response.message
      message: exception instanceof HttpException ? exception.getResponse() : exception.message,
    };
    const resp: FastifyReply = host.switchToHttp().getResponse();
    // for some reason, eslint thinks below is a promise, which is not.
    // eslint-disable-next-line @typescript-eslint/no-floating-promises
    resp.status(statusCode).send(messageBody);
  }
}
