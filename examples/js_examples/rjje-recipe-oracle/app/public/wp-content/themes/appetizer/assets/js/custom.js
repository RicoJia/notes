(function($) {
  'use strict';

    // ScrollUp
    $(window).on('scroll', function () {
      if ($(this).scrollTop() > 200) {
        $('.scrollingUp').addClass('is-active');
      } else {
        $('.scrollingUp').removeClass('is-active');
      }
    });
    // Sticky Header
    $(window).on('scroll', function() {
      if ($(window).scrollTop() >= 250) {
          $('.is-sticky-on').addClass('is-sticky-menu');
      }
      else {
          $('.is-sticky-on').removeClass('is-sticky-menu');
      }
    });
    $( document ).ready(function() {
        $('.scrollingUp').on('click', function () {
          $("html, body").animate({
            scrollTop: 0
          }, 600);
          return false;
        });
    });
}(jQuery));
